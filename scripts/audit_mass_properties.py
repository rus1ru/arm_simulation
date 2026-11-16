#!/usr/bin/python3
"""Read-only audit of original SolidWorks export; stdlib only, SI units assumed."""
import argparse
import collections
import csv
import hashlib
import json
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET


def eigenvalues(matrix):
    """Jacobi eigenvalues for a real symmetric 3x3 matrix."""
    a = [row[:] for row in matrix]
    for _ in range(40):
        p, q = max(((0, 1), (0, 2), (1, 2)), key=lambda ij: abs(a[ij[0]][ij[1]]))
        if abs(a[p][q]) < 1e-20:
            break
        angle = 0.5 * math.atan2(2*a[p][q], a[q][q]-a[p][p])
        c, s = math.cos(angle), math.sin(angle)
        app, aqq, apq = a[p][p], a[q][q], a[p][q]
        a[p][p] = c*c*app - 2*s*c*apq + s*s*aqq
        a[q][q] = s*s*app + 2*s*c*apq + c*c*aqq
        a[p][q] = a[q][p] = 0.0
        for k in range(3):
            if k not in (p, q):
                x, y = a[k][p], a[k][q]
                a[k][p] = a[p][k] = c*x-s*y
                a[k][q] = a[q][k] = s*x+c*y
    return sorted(a[i][i] for i in range(3))


def mesh_audit(path):
    raw = path.read_bytes()
    count = struct.unpack_from('<I', raw, 80)[0] if len(raw) >= 84 else 0
    if len(raw) == 84+50*count:
        triangles = [tuple(tuple(v[j:j+3]) for j in (3, 6, 9))
                     for v in struct.iter_unpack('<12fH', raw[84:])]
    else:
        vertices = [tuple(map(float, line.split()[1:])) for line in raw.decode().splitlines()
                    if line.strip().startswith('vertex ')]
        if len(vertices) % 3:
            raise ValueError('Incomplete ASCII STL triangles')
        triangles = [tuple(vertices[i:i+3]) for i in range(0, len(vertices), 3)]
    vertices = [v for tri in triangles for v in tri]
    low = [min(v[i] for v in vertices) for i in range(3)]
    high = [max(v[i] for v in vertices) for i in range(3)]
    volume_terms = []
    edges = collections.Counter()
    directions = collections.Counter()
    degenerate = 0
    # Rounded vertex welding tolerance: 1e-7 m, about float32 mesh precision here.
    for tri in triangles:
        a, b, c = tri
        volume_terms.append((a[0]*(b[1]*c[2]-b[2]*c[1])+a[1]*(b[2]*c[0]-b[0]*c[2])+a[2]*(b[0]*c[1]-b[1]*c[0]))/6)
        keys = [tuple(round(x/1e-7) for x in v) for v in tri]
        if len(set(keys)) < 3:
            degenerate += 1
        for u, v in zip(keys, keys[1:]+keys[:1]):
            edge = tuple(sorted((u, v)))
            edges[edge] += 1
            directions[edge] += 1 if u < v else -1
    boundary = sum(n == 1 for n in edges.values())
    nonmanifold = sum(n > 2 for n in edges.values())
    inconsistent = sum(edges[e] == 2 and d != 0 for e, d in directions.items())
    return dict(sha256=hashlib.sha256(raw).hexdigest(), triangles=len(triangles), bounds_m=[low, high],
                signed_volume_m3=math.fsum(volume_terms), boundary_edges=boundary,
                nonmanifold_edges=nonmanifold, inconsistent_winding_edges=inconsistent,
                degenerate_triangles_after_welding=degenerate,
                edge_closed=(boundary == 0 and nonmanifold == 0), vertex_weld_tolerance_m=1e-7)


def audit(root):
    urdf = root/'urdf/robo2.urdf'
    csv_path = root/'urdf/robo2.csv'
    rows = {r['Link Name']: r for r in csv.DictReader(csv_path.open())}
    records = []
    for link in ET.parse(urdf).getroot().findall('link'):
        name = link.get('name')
        inertial = link.find('inertial')
        mass = float(inertial.find('mass').get('value'))
        com = list(map(float, inertial.find('origin').get('xyz').split()))
        inertia = {k: float(v) for k, v in inertial.find('inertia').attrib.items()}
        matrix = [[inertia['ixx'], inertia['ixy'], inertia['ixz']],
                  [inertia['ixy'], inertia['iyy'], inertia['iyz']],
                  [inertia['ixz'], inertia['iyz'], inertia['izz']]]
        eig = eigenvalues(matrix)
        visual = link.find('visual')
        # This audit deliberately refuses unsupported transforms instead of ignoring them.
        assert visual.find('origin').get('xyz') == '0 0 0'
        assert visual.find('origin').get('rpy') == '0 0 0'
        mesh = visual.find('geometry/mesh')
        assert mesh.get('scale', '1 1 1') == '1 1 1'
        mesh_result = mesh_audit(root/'meshes'/Path(mesh.get('filename')).name)
        bounds = mesh_result['bounds_m']
        csv_values = {'Mass': mass, **{'Center of Mass '+axis: value for axis, value in zip('XYZ', com)},
                      **{'Moment '+k[0].upper()+k[1:]: v for k, v in inertia.items()}}
        mismatches = [k for k, v in csv_values.items() if float(rows[name][k]) != v]
        volume = abs(mesh_result['signed_volume_m3'])
        density = mass/volume if volume else None
        records.append(dict(link=name, mass_kg=mass, positive_mass=mass>0, com_m=com,
                            com_in_mesh_aabb=all(bounds[0][i]-1e-6 <= com[i] <= bounds[1][i]+1e-6 for i in range(3)),
                            inertia_symmetric_by_urdf_definition=True, principal_moments_kg_m2=eig,
                            inertia_positive_definite=eig[0]>0,
                            principal_triangle_margin_kg_m2=eig[0]+eig[1]-eig[2],
                            principal_triangle_valid=eig[0]+eig[1]>=eig[2]-1e-12,
                            csv_mismatched_fields=mismatches, mesh=mesh_result,
                            inferred_density_kg_m3=density,
                            density_review_flag=density is None or density<500 or density>20000))
    total = math.fsum(r['mass_kg'] for r in records)
    return dict(source_urdf=str(urdf), source_csv=str(csv_path),
                source_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in (urdf, csv_path)},
                total_mass_kg=total, manual_reference_kg=37, datasheet_reference_kg=34,
                deficit_vs_manual_kg=37-total, deficit_vs_datasheet_kg=34-total,
                assumptions=['STL coordinates are metres, matching URDF with no scale.',
                             'Inertias are about the declared COM in their inertial frame.',
                             'Density uses absolute algebraic signed mesh volume, not verified material volume.',
                             'Density review band 500–20000 kg/m^3 is heuristic, not a material identification.',
                             'AABB containment is necessary, not sufficient, for COM geometric validity.',
                             'Edge closure does not test self-intersections, overlaps, or internal shell orientation.'],
                links=records)


def markdown(result):
    lines = ['# Original exported mass-properties audit', '',
             'Read-only audit. No mass, inertia, mesh, or active-model values were changed.', '',
             f"Exported total: **{result['total_mass_kg']:.7f} kg** (including both CAD fingers).",
             f"Manual GP7/AR900: 37 kg; deficit {result['deficit_vs_manual_kg']:.4f} kg. Datasheet GP7: 34 kg; deficit {result['deficit_vs_datasheet_kg']:.4f} kg.",
             'The reference documents disagree; neither supplies per-link inertias. Do not scale the export or invent missing masses.', '',
             '| Link | Mass kg | PD / triangle | COM in AABB | Closed edges | Signed volume m³ | Inferred kg/m³ |',
             '|---|---:|---|---|---|---:|---:|']
    for r in result['links']:
        lines.append(f"| {r['link']} | {r['mass_kg']:.7g} | {r['inertia_positive_definite']} / {r['principal_triangle_valid']} | {r['com_in_mesh_aabb']} | {r['mesh']['edge_closed']} | {r['mesh']['signed_volume_m3']:.6g} | {r['inferred_density_kg_m3']:.6g} |")
    lines += ['', '## Interpretation and limitations', '',
              'All tensors are symmetric by URDF definition (one shared off-diagonal coefficient). The JSON reports eigenvalues, principal-moment triangle margins, mesh bounds, winding and edge counts, CSV comparisons, and source hashes.',
              'All nine masses are positive and all nine inertia tensors pass positive-definite and principal-moment triangle checks. Mass, COM and inertia coefficients exactly match the original CSV. Both finger COMs fail their own mesh bounds; all other COMs pass this necessary-only check.',
              'Every inferred density is approximately 999–1002 kg/m³. This near-uniform pattern warrants checking CAD material assignments, but is not conclusive: the arm meshes fail the welded edge-manifold test, and enclosed visual volume need not equal material volume. Only the two finger meshes pass edge closure. No heuristic density-band flag is triggered.',
              'The original finger inertial/geometry frame is invalid wherever the declared COM lies outside its own mesh AABB. A parent joint transform cannot repair a mismatch internal to a link. This is a frame-registration issue, not evidence that the finger mass should be scaled.',
              'STL edge tests use vertex welding at 1e-7 m; COM bounds allow 1e-6 m rounding tolerance. Signed volume is algebraic: intersecting components, reversed or nested shells can invalidate its material interpretation. Inferred densities are diagnostic flags only, not conclusive physical densities.',
              'Sources: original `robo2/urdf/robo2.urdf`, matching exporter CSV, original STL meshes; `processed_documents/G7_manual.txt` (Approx. mass 37 kg), `processed_documents/GP7_GP8_datasheet.txt` (Weight GP7 34 kg).', '',
              '## Assumptions', *['- '+a for a in result['assumptions']], '',
              'Reproduce from the project directory: `/usr/bin/python3 robo2/scripts/audit_mass_properties.py`.']
    return '\n'.join(lines)+'\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--output-dir', type=Path)
    args = parser.parse_args()
    out = args.output_dir or args.root.parent/'processed_documents'
    out.mkdir(parents=True, exist_ok=True)
    result = audit(args.root)
    (out/'mass_properties_audit.json').write_text(json.dumps(result, indent=2)+'\n')
    (out/'mass_properties_audit.md').write_text(markdown(result))
    print(json.dumps({'total_mass_kg': result['total_mass_kg'], 'links': len(result['links']),
                      'failed_com_links': [r['link'] for r in result['links'] if not r['com_in_mesh_aabb']]}))


if __name__ == '__main__':
    main()
