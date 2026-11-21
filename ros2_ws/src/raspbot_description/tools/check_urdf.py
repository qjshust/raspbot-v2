"""URDF 静态检查（离线，无 ROS 依赖）：XML 良构性 + joint/link 引用完整性。

用法：python tools/check_urdf.py [path/to/urdf]
默认检查本包 urdf/raspbot.urdf；检查失败返回非零退出码。
"""

import os
import sys
import xml.etree.ElementTree as ET


def check_urdf(path):
    errors = []

    # 1) XML 良构性
    try:
        tree = ET.parse(path)
    except ET.ParseError as e:
        print('[FAIL] URDF XML 解析失败: %s' % e)
        return 1
    robot = tree.getroot()
    if robot.tag != 'robot':
        errors.append('根元素必须是 <robot>，收到 <%s>' % robot.tag)

    # 2) link / joint 收集与引用检查
    links = {el.attrib.get('name') for el in robot.findall('link')}
    joints = robot.findall('joint')
    for j in joints:
        name = j.attrib.get('name', '<unnamed>')
        jtype = j.attrib.get('type')
        parent = j.findtext('parent', default='').strip()
        child = j.findtext('child', default='').strip()
        if jtype not in ('fixed', 'revolute', 'continuous', 'prismatic', 'floating', 'planar'):
            errors.append('joint %s: 非法类型 %r' % (name, jtype))
        if parent and parent not in links:
            errors.append('joint %s: parent link %r 不存在' % (name, parent))
        if child and child not in links:
            errors.append('joint %s: child link %r 不存在' % (name, child))
        origin = j.find('origin')
        if origin is None:
            errors.append('joint %s: 缺少 <origin>（fixed joint 必须显式给出）' % name)
        else:
            xyz = origin.attrib.get('xyz', '0 0 0').split()
            if len(xyz) != 3:
                errors.append('joint %s: origin xyz 需 3 个数值' % name)
            else:
                try:
                    [float(v) for v in xyz]
                except ValueError:
                    errors.append('joint %s: origin xyz 非数值 %r' % (name, xyz))

    # 3) 每个 link 都有 inertial（URDF 规范友好）
    for el in robot.findall('link'):
        name = el.attrib.get('name')
        if el.find('inertial') is None:
            errors.append('link %s: 缺少 <inertial>' % name)

    # 4) 无重复命名
    seen = {}
    for el in robot.findall('link') + joints:
        name = el.attrib.get('name')
        if name in seen:
            errors.append('重复命名: %s (%s 与 %s)' % (name, seen[name], el.tag))
        seen[name] = el.tag

    if errors:
        for e in errors:
            print('[FAIL] ' + e)
        return 1

    print('[OK] %s: %d links, %d joints, 引用完整' %
          (path, len(links), len(joints)))
    return 0


def main():
    if len(sys.argv) > 1:
        path = sys.argv[1]
    else:
        here = os.path.dirname(os.path.abspath(__file__))
        path = os.path.normpath(os.path.join(here, '..', 'urdf', 'raspbot.urdf'))
    if not os.path.isfile(path):
        print('[FAIL] 文件不存在: %s' % path)
        return 1
    return check_urdf(path)


if __name__ == '__main__':
    raise SystemExit(main())
