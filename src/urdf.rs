/*use nalgebra::{convert, OPoint, UnitVector3, Vector3, U3};
use parry3d::mass_properties::MassProperties;
use xurdf::{Link, Joint};
use rapier3d_urdf::{UrdfRobot, UrdfLoaderOptions, UrdfMultibodyOptions};

fn xurdf_to_massproperties(link: &Link) -> MassProperties {
    let local_com:OPoint<f32, U3> = OPoint::<f32, U3>::new(
        link.inertial.origin.xyz[0] as f32,
        link.inertial.origin.xyz[1] as f32,
        link.inertial.origin.xyz[2] as f32
    );
    MassProperties::with_inertia_matrix(
        local_com,
        link.inertial.mass as f32,
        convert(link.inertial.inertia)
    )
}

fn xurdf_joint_to_revolute(joint: &Joint) -> RevoluteJoint {
    let axis = UnitVector3::<f32>::new_normalize(convert(joint.axis));
    let jt = RevoluteJointBuilder::new(axis)
        .local_anchor1(point![
            joint.origin.xyz[0] as f32,
            joint.origin.xyz[1] as f32,
            joint.origin.xyz[2] as f32,
        ]).build();
    jt
}


pub fn load_urdf(urdf_path: &Path) -> (Vec<Link>, Vec<Joint>) {
    let res = parse_urdf_from_file(urdf_path);
*/
