import xml.etree.ElementTree as ET

def print_urdf_movable_joints(urdf_path):
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    print("Movable joints in URDF (exclude fixed):\n")

    for joint in root.findall("joint"):
        joint_type = joint.get("type")

        # 跳過 fixed joint
        if joint_type == "fixed":
            continue

        name = joint.get("name")
        parent = joint.find("parent").get("link")
        child = joint.find("child").get("link")

        axis_elem = joint.find("axis")
        axis = axis_elem.get("xyz") if axis_elem is not None else "N/A"

        limit_elem = joint.find("limit")
        if limit_elem is not None:
            lower = limit_elem.get("lower", "N/A")
            upper = limit_elem.get("upper", "N/A")
        else:
            lower = upper = "N/A"

        print(f"Joint name : {name}")
        print(f"  type     : {joint_type}")
        print(f"  parent   : {parent}")
        print(f"  child    : {child}")
        print(f"  axis     : {axis}")
        print(f"  limit    : [{lower}, {upper}]")
        print("-" * 40)


if __name__ == "__main__":
    urdf_path = "/home/csl/yale_ws/robot_lab/source/robot_lab/data/Robots/bigreddog/urdf/bigreddog.urdf"
    print_urdf_movable_joints(urdf_path)
