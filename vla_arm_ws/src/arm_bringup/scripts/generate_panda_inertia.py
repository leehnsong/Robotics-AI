#!/usr/bin/env python3
"""
generate_panda_inertia.py

moveit_resources 의 panda.urdf 는 MoveIt 경로계획 테스트 전용이라 <inertial>(질량/관성)
정보가 전혀 없다. 그대로 Gazebo 에 스폰하면 질량 없는 링크들이 처리되지 못해
관절이 전부 사라진다("Skipping joint ... not in the gazebo model").

이 스크립트는 원본 panda.urdf 를 읽어, <inertial> 이 없는 모든 <link> 에
시뮬레이션용 최소 관성을 주입한 panda_inertia.urdf 를 생성한다.
(position 명령 인터페이스는 위치를 직접 세팅하므로 관성 값의 정확도는 중요하지 않다.)

사용:
  python3 generate_panda_inertia.py \
      <원본 panda.urdf 경로> <출력 panda_inertia.urdf 경로>
"""
import sys
import xml.etree.ElementTree as ET


def add_inertials(in_path: str, out_path: str) -> int:
    tree = ET.parse(in_path)
    root = tree.getroot()
    added = 0
    for link in root.findall('link'):
        if link.find('inertial') is not None:
            continue
        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'mass', {'value': '1.0'})
        ET.SubElement(inertial, 'origin', {'xyz': '0 0 0', 'rpy': '0 0 0'})
        ET.SubElement(inertial, 'inertia', {
            'ixx': '0.01', 'ixy': '0', 'ixz': '0',
            'iyy': '0.01', 'iyz': '0', 'izz': '0.01',
        })
        added += 1
    tree.write(out_path, encoding='unicode', xml_declaration=True)
    return added


if __name__ == '__main__':
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(1)
    n = add_inertials(sys.argv[1], sys.argv[2])
    print(f'inertial 추가된 링크 수: {n} -> {sys.argv[2]}')
