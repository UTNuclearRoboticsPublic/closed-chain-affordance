#include <Eigen/Core>
#include <affordance_util/affordance_util.hpp>
#include <filesystem>
#include <iomanip> // for std::precision
#include <iostream>
#include <fstream> // To read files

int main()
{

    using namespace affordance_util;
    // Precision for testing
    std::cout << std::fixed << std::setprecision(4); // Display up to 4 decimal places

    std::cout << "Testing VecToSo3" << std::endl;
    const Eigen::Vector3d omg(1.0, 2.0, 3.0);
    std::cout << "Input: \n" << omg << std::endl;
    const Eigen::Matrix3d &so3mat = VecToso3(omg);
    std::cout << "Output: \n" << so3mat << std::endl;

    std::cout << "\nTesting so3ToVec" << std::endl;
    std::cout << "Input: \n" << so3mat << std::endl;
    std::cout << "Output: \n" << so3ToVec(so3mat) << std::endl;

    std::cout << "\nTesting VecTose3" << std::endl;
    const Eigen::VectorXd V = (Eigen::VectorXd(6) << 1, 2, 3, 4, 5, 6).finished();
    std::cout << "Input: \n" << V << std::endl;
    std::cout << "Output: \n" << VecTose3(V) << std::endl;

    std::cout << "\nTesting AxisAng3" << std::endl;
    const Eigen::Vector3d expc3 = (Eigen::Vector3d() << 1, 2, 3).finished();
    std::cout << "Input: \n" << expc3 << std::endl;
    const auto &[expc3Axis, expc3Angle] = AxisAng3(expc3);
    std::cout << "Output axis: \n" << expc3Axis << std::endl;
    std::cout << "Output angle: \n" << expc3Angle << std::endl;

    std::cout << "\nTesting MatrixExp3" << std::endl;
    std::cout << "Input: \n" << so3mat << std::endl;
    std::cout << "Output: \n" << MatrixExp3(so3mat) << std::endl;

    const Eigen::Matrix4d se3mat =
        (Eigen::Matrix4d() << 0, 0, 0, 0, 0, 0, -1.5708, 2.3562, 0, 1.5708, 0, 2.3562, 0, 0, 0, 0).finished();
    std::cout << "\nTesting MatrixExp6" << std::endl;
    std::cout << "Input: \n" << se3mat << std::endl;
    std::cout << "Output: \n" << MatrixExp6(se3mat) << std::endl;

    const Eigen::Matrix4d M = (Eigen::Matrix4d() << -1, 0, 0, 0, 0, 1, 0, 6, 0, 0, -1, 2, 0, 0, 0, 1).finished();

    Eigen::MatrixXd Slist(6, 3);
    Slist << 0, 0, 0, 0, 0, 0, 1, 0, -1, 4, 0, -6, 0, 1, 0, 0, 0, -0.1;

    Eigen::Vector3d thetalist = (Eigen::Vector3d() << M_PI / 2, 3, M_PI).finished();
    std::cout << "\nTesting FKinSpace" << std::endl;
    std::cout << "Input M: \n" << M << std::endl;
    std::cout << "Input Slist: \n" << Slist << std::endl;
    std::cout << "Input thetalist: \n" << thetalist << std::endl;
    std::cout << "Output: \n" << FKinSpace(M, Slist, thetalist) << std::endl;

    Eigen::MatrixXd Slist2(6, 4);

    Slist2 << 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 2, 0, 0.2, 0.2, 0, 2, 0.3, 0.2, 3, 1, 0.4;
    Eigen::VectorXd thetalist2(4);
    thetalist2 << 0.2, 1.1, 0.1, 1.2;
    std::cout << "\nTesting JacobianSpace" << std::endl;
    std::cout << "Input Slist: \n" << Slist2 << std::endl;
    std::cout << "Input thetalist: \n" << thetalist2 << std::endl;
    std::cout << "Output: \n" << JacobianSpace(Slist2, thetalist2) << std::endl;

    Eigen::Matrix4d htm;
    htm << 1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 3, 0, 0, 0, 1;
    std::cout << "\nTesting Adjoint" << std::endl;
    std::cout << "Input htm: \n" << htm << std::endl;
    std::cout << "Output: \n" << Adjoint(htm) << std::endl;

    std::cout << "\nTesting TransInv" << std::endl;
    std::cout << "Input htm: \n" << htm << std::endl;
    std::cout << "Output: \n" << TransInv(htm) << std::endl;

    Eigen::Matrix4d se3mat2;
    se3mat2 << 0, -3, 2, 4, 3, 0, -1, 5, -2, 1, 0, 6, 0, 0, 0, 0;
    std::cout << "\nTesting se3ToVec" << std::endl;
    std::cout << "Input se3mat: \n" << se3mat2 << std::endl;
    std::cout << "Output: \n" << se3ToVec(se3mat2) << std::endl;

    const Eigen::Matrix3d R = (Eigen::Matrix3d() << 0, 0, 1, 1, 0, 0, 0, 1, 0).finished();
    std::cout << "\nTesting MatrixLog3" << std::endl;
    std::cout << "Input R: \n" << R << std::endl;
    std::cout << "Output: \n" << MatrixLog3(R) << std::endl;

    const double near = -1e-7;
    std::cout << "\nTesting NearZero" << std::endl;
    std::cout << "Input near: \n" << near << std::endl;
    std::cout << "Output: \n" << NearZero(near) << std::endl;

    const Eigen::Matrix4d T = (Eigen::Matrix4d() << 1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 3, 0, 0, 0, 1).finished();
    std::cout << "\nTesting MatrixLog6" << std::endl;
    std::cout << "Input T: \n" << T << std::endl;
    std::cout << "Output: \n" << MatrixLog6(T) << std::endl;

    const double spot_tolerance = 1e-2;   // tolerance for urdf and yaml comparison
    const double kinova_tolerance = 1e-3; // tolerance for urdf and yaml comparison
    //-------------------------------------------------------------------------------------------
    std::cout << "\nTesting robot_builder Spot YAML version" << std::endl;

    // Get path to test directory where robot test files are
    const std::filesystem::path current_file_path(__FILE__);
    const std::filesystem::path project_root = current_file_path.parent_path();
    std::filesystem::path spot_yaml_path = project_root / "test" / "cca_spot_description.yaml";
    RobotConfig spot_yaml_robot_config;
    Eigen::MatrixXd spot_yaml_slist;
    Eigen::MatrixXd spot_yaml_m_out;
    try
    {
        spot_yaml_robot_config = robot_builder(spot_yaml_path.string());
        std::cout << "Input filepath: \n" <<spot_yaml_path.string()<< std::endl;
        spot_yaml_slist = spot_yaml_robot_config.Slist;
        spot_yaml_m_out = spot_yaml_robot_config.M;
        std::cout << "Output Slist: \n" << spot_yaml_slist << std::endl;
        std::cout << "Output M: \n" << spot_yaml_m_out << std::endl;
        std::cout << "Output ref_frame_name: \n" << spot_yaml_robot_config.frame_names.ref << std::endl;
        std::cout << "Output tool_name: \n" << spot_yaml_robot_config.frame_names.tool << std::endl;
        std::cout << "Output tool offset from EE: \n"
                  << spot_yaml_robot_config.ee_to_tool_offset.transpose() << std::endl;
        std::cout << "Output joint_names: ";
        for (const std::string &joint_name : spot_yaml_robot_config.joint_names.robot)
        {
            std::cout << joint_name << ",";
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "***Error building robot: " << e.what() << std::endl;
    }
    std::filesystem::path spot_urdf_path = project_root / "test" / "spot.urdf";
    std::string spot_urdf_string;
    {
        std::ifstream file(spot_urdf_path.string());
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open Spot URDF file");
        }
        std::stringstream buffer;
        buffer << file.rdbuf();
        spot_urdf_string = buffer.str();
    }
    std::cout << "\nTesting robot_builder Spot URDF version" << std::endl;
    try
    {
        std::filesystem::path spot_urdf_yaml_path = project_root / "test" / "cca_spot_urdf.yaml";
	auto spot_urdf_robot_builder_info = extract_info_for_urdf_robot_builder(spot_urdf_yaml_path.string());
        auto robot_config = robot_builder(spot_urdf_string, spot_urdf_robot_builder_info);
        std::cout << "Input filepath: \n" << spot_urdf_path.string() << std::endl;
        std::cout << "Input ref_frame: \n" << spot_urdf_robot_builder_info.frame_names.ref << std::endl;
        std::cout << "Input base_joint: \n" << spot_urdf_robot_builder_info.kinematic_chain.base_joint_name << std::endl;
        std::cout << "Input end_joint: \n" << spot_urdf_robot_builder_info.kinematic_chain.end_joint_name << std::endl;
        std::cout << "Input ee_frame: \n" << spot_urdf_robot_builder_info.frame_names.ee << std::endl;
        std::cout << "Input ee_to_tool_offset: \n" << spot_urdf_robot_builder_info.ee_to_tool_offset.transpose() << std::endl;
        auto urdf_slist = robot_config.Slist;
        auto urdf_m_out = robot_config.M;
        std::cout << "Output Slist: \n" << urdf_slist << std::endl;
        std::cout << "Output M with Tool Location: \n" << urdf_m_out << std::endl;
        bool slist_equal =
            spot_yaml_slist.isApprox(urdf_slist, spot_tolerance); // Just compare solution, excluding affordance
        if (slist_equal)
        {
            std::cout << "The URDF and YAML Slist match within " << spot_tolerance << std::endl;
        }
        else
        {

            std::cerr << "The URDF and YAML Slist do not match within " << spot_tolerance << std::endl;
        }
        bool m_out_equal =
            spot_yaml_m_out.isApprox(urdf_m_out, spot_tolerance); // Just compare solution, excluding affordance
        if (m_out_equal)
        {
            std::cout << "The URDF and YAML M matrix match within " << spot_tolerance << std::endl;
        }
        else
        {

            std::cerr << "The URDF and YAML M matrix do not match within " << spot_tolerance <<". This may have to do with the urdf EE frame orientation being different from what's assumed in the YAML version."<< std::endl;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error building robot: " << e.what() << std::endl;
    }

    //--------------------------------------------------------------------------------------------------------
    std::cout << "\nTesting robot_builder Kinova YAML version" << std::endl;

    auto kinova_yaml_path = project_root / "test" / "cca_kinova_gen3_7dof_description.yaml";
    RobotConfig kinova_yaml_robot_config;
    Eigen::MatrixXd kinova_yaml_slist;
    Eigen::MatrixXd kinova_yaml_m_out;
    try
    {
        kinova_yaml_robot_config = robot_builder(kinova_yaml_path.string());
        std::cout << "Input filepath: \n" <<kinova_yaml_path.string()<< std::endl;
        kinova_yaml_slist = kinova_yaml_robot_config.Slist;
        kinova_yaml_m_out = kinova_yaml_robot_config.M;
        std::cout << "Output Slist: \n" << kinova_yaml_slist << std::endl;
        std::cout << "Output M: \n" << kinova_yaml_m_out << std::endl;
        std::cout << "Output ref_frame_name: \n" << kinova_yaml_robot_config.frame_names.ref << std::endl;
        std::cout << "Output tool_name: \n" << kinova_yaml_robot_config.frame_names.tool << std::endl;
        std::cout << "Output tool offset from EE: \n"
                  << kinova_yaml_robot_config.ee_to_tool_offset.transpose() << std::endl;
        std::cout << "Output joint_names: ";
        for (const std::string &joint_name : kinova_yaml_robot_config.joint_names.robot)
        {
            std::cout << joint_name << ",";
        }
        std::cout << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "***Error building robot: " << e.what() << std::endl;
    }

    // Get urdf filepath
    std::filesystem::path kinova_urdf_path = project_root / "test" / "kinova_gen3_7dof.urdf";
    std::string kinova_urdf_string;
    {
        std::ifstream file(kinova_urdf_path.string());
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open kinova URDF file");
        }
        std::stringstream buffer;
        buffer << file.rdbuf();
        kinova_urdf_string = buffer.str();
    }

    // Convert to string for file operations
    std::cout << "\nTesting robot_builder  Kinova URDF version" << std::endl;
    try
    {
        std::filesystem::path kinova_urdf_yaml_path = project_root / "test" / "cca_kinova_gen3_7dof_urdf.yaml";
	auto kinova_urdf_robot_builder_info = extract_info_for_urdf_robot_builder(kinova_urdf_yaml_path.string());
        auto robot_config = robot_builder(kinova_urdf_string, kinova_urdf_robot_builder_info);
        std::cout << "Input filepath: \n" << kinova_urdf_path.string() << std::endl;
        std::cout << "Input ref_frame: \n" << kinova_urdf_robot_builder_info.frame_names.ref << std::endl;
        std::cout << "Input base_joint: \n" << kinova_urdf_robot_builder_info.kinematic_chain.base_joint_name << std::endl;
        std::cout << "Input end_joint: \n" << kinova_urdf_robot_builder_info.kinematic_chain.end_joint_name << std::endl;
        std::cout << "Input ee_frame: \n" << kinova_urdf_robot_builder_info.frame_names.ee << std::endl;
        std::cout << "Input ee_to_tool_offset: \n" << kinova_urdf_robot_builder_info.ee_to_tool_offset.transpose() << std::endl;
        auto urdf_slist = robot_config.Slist;
        auto urdf_m_out = robot_config.M;
        std::cout << "Output Slist: \n" << urdf_slist << std::endl;
        std::cout << "Output M with tool location: \n" << urdf_m_out << std::endl;
        bool slist_equal =
            kinova_yaml_slist.isApprox(urdf_slist, kinova_tolerance); // Just compare solution, excluding affordance
        if (slist_equal)
        {
            std::cout << "The URDF and YAML Slist match within " << kinova_tolerance << std::endl;
        }
        else
        {

            std::cerr << "The URDF and YAML Slist do not match within " << kinova_tolerance << std::endl;
        }
        bool m_out_equal =
            kinova_yaml_m_out.isApprox(urdf_m_out, kinova_tolerance); // Just compare solution, excluding affordance
        if (m_out_equal)
        {
            std::cout << "The URDF and YAML M matrix match within " << kinova_tolerance << std::endl;
        }
        else
        {

            std::cerr << "The URDF and YAML M matrix do not match within " << kinova_tolerance << ". This may have to do with the urdf EE frame orientation being different from what's assumed in the YAML version." << std::endl;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error building robot: " << e.what() << std::endl;
    }
    //--------------------------------------------------------------------------------------------------------

    ScrewInfo si;
    si.type = affordance_util::ScrewType::TRANSLATION;
    si.axis << 1.0, 0.0, 0.0;
    std::cout << "\n Testing get_screw" << std::endl;
    std::cout << "\n Translation input axis: " << si.axis.transpose() << std::endl;
    std::cout << "\n Translation output: " << get_screw(si).transpose() << std::endl;
    si.screw = get_screw(si);
    std::cout << "\n Testing get_axis_from_screw" << std::endl;
    std::cout << "\n Axis output for above screw: " << get_axis_from_screw(si).transpose() << std::endl;
    si = ScrewInfo();
    si.type = affordance_util::ScrewType::SCREW;
    si.axis << 0.0, 1.0, 0.0;
    si.location << 1.0, 2.0, 3.0;
    si.pitch = 0.2;
    std::cout << "\n Testing get_screw" << std::endl;
    std::cout << "\n Screw motion input axis: " << si.axis.transpose() << std::endl;
    std::cout << "\n Screw motion axis location: " << si.location.transpose() << std::endl;
    std::cout << "\n Screw motion pitch: " << si.pitch << std::endl;
    std::cout << "\n Screw motion output: " << get_screw(si).transpose() << std::endl;
    si.screw = get_screw(si);
    std::cout << "\n Testing get_axis_from_screw" << std::endl;
    std::cout << "\n Axis output for above screw: " << get_axis_from_screw(si).transpose() << std::endl;
    si = ScrewInfo();
    si.type = affordance_util::ScrewType::ROTATION;
    si.axis << 0.0, 0.0, 1.0;
    si.location << 1.0, 2.0, 3.0;
    std::cout << "\n Testing get_screw" << std::endl;
    std::cout << "\n Rotation input axis: " << si.axis.transpose() << std::endl;
    std::cout << "\n Rotation axis location: " << si.location.transpose() << std::endl;
    std::cout << "\n Rotation output: " << get_screw(si).transpose() << std::endl;
    si.screw = get_screw(si);
    std::cout << "\n Testing get_axis_from_screw" << std::endl;
    std::cout << "\n Axis output for above screw: " << get_axis_from_screw(si).transpose() << std::endl;

    Eigen::Vector3d screw_axis(1.0, 0.0, 0.0);
    Eigen::Vector3d screw_location(1.0, 2.0, 3.0);
    std::cout << "\n Testing get_screw rotation overload" << std::endl;
    std::cout << "\n Rotation input axis: " << screw_axis.transpose() << std::endl;
    std::cout << "\n Rotation axis location: " << screw_location.transpose() << std::endl;
    std::cout << "\n Rotation output: " << get_screw(screw_axis, screw_location).transpose() << std::endl;

    double gripper_start_state = 0;
    double gripper_end_state = -0.5;
    int trajectory_density = 5;
    affordance_util::GripperGoalType gripper_goal_type = affordance_util::GripperGoalType::CONSTANT;
    std::cout << "\n Testing compute_gripper_joint_trajectory " << std::endl;
    std::cout << "\n Gripper start state: " << gripper_start_state << std::endl;
    std::cout << "\n Gripper end state: " << gripper_end_state << std::endl;
    std::cout << "\n Trajectory density: " << trajectory_density << std::endl;
    std::cout << "\n Gripper goal type: CONSTANT" << std::endl;
    std::vector<double> gripper_trajectory =
        compute_gripper_joint_trajectory(gripper_goal_type, gripper_start_state, gripper_end_state, trajectory_density);
    std::cout << "\nFunction output:" << std::endl;
    for (const double &point : gripper_trajectory)
    {
        std::cout << point << ",";
    }
    std::cout << std::endl;

    gripper_goal_type = affordance_util::GripperGoalType::CONTINUOUS;
    std::cout << "\n Testing compute_gripper_joint_trajectory " << std::endl;
    std::cout << "\n Gripper start state: " << gripper_start_state << std::endl;
    std::cout << "\n Gripper end state: " << gripper_end_state << std::endl;
    std::cout << "\n Trajectory density: " << trajectory_density << std::endl;
    std::cout << "\n Gripper goal type: CONTINUOUS" << std::endl;
    gripper_trajectory =
        compute_gripper_joint_trajectory(gripper_goal_type, gripper_start_state, gripper_end_state, trajectory_density);
    std::cout << "\nFunction output:" << std::endl;
    for (const double &point : gripper_trajectory)
    {
        std::cout << point << ",";
    }
    std::cout << std::endl;

    {
    	std::cout << "\nTesting compute_se3_screw_trajectory " << std::endl;
    	// Define a simple screw: translation along the z-axis 
    	affordance_util::ScrewInfo si;
	si.type = affordance_util::ScrewType::TRANSLATION;
    	si.axis = Eigen::Vector3d(0, 0, 1);              // unit axis along z
    	si.location  = Eigen::Vector3d(0, 0, 0);              // passes through origin

    	// Motion parameters
    	double theta_total = 0.5;   // half a meter
    	int trajectory_density = 5;

    	// Start pose at identity
    	Eigen::Matrix4d T_start = Eigen::Matrix4d::Identity();

    	// Compute discretized SE(3) trajectory
    	const std::vector<Eigen::Matrix4d> T_traj =
    	    affordance_util::compute_se3_screw_trajectory(si, theta_total, trajectory_density, T_start);

    	// Print the resulting transforms
    	for (size_t i = 0; i < T_traj.size(); ++i)
    	{
    	  std::cout << "T[" << i << "] =\n" << T_traj[i] << "\n\n";
    	}
    }
    {
        std::cout << "\nTesting axis_to_vec\n" << std::endl;
    
        std::vector<std::pair<Axis, std::string>> test_axes = {
            {Axis::X, "X"},
            {Axis::Y, "Y"},
            {Axis::Z, "Z"},
            {Axis::X_MINUS, "X_MINUS"},
            {Axis::Y_MINUS, "Y_MINUS"},
            {Axis::Z_MINUS, "Z_MINUS"},
            {Axis::MANUAL, "MANUAL"}
        };
    
        for (const auto &[axis_enum, axis_name] : test_axes)
        {
            std::cout << "Axis: " << axis_name << " -> ";
            try
            {
                Eigen::Vector3d vec = axis_to_vec(axis_enum);
                std::cout << vec.transpose() << std::endl;
            }
            catch (const std::exception &e)
            {
                std::cout << "Error: " << e.what() << std::endl;
            }
        }
    }

    {
    	std::cout << "\nTesting get_affordance_info_from_fk" << std::endl;

        // We'll use Spot's robot description
        affordance_util::RobotDescription robot_description;
        try {
            std::filesystem::path spot_urdf_yaml_path = project_root / "test" / "cca_spot_urdf.yaml";
	    auto spot_urdf_robot_builder_info = extract_info_for_urdf_robot_builder(spot_urdf_yaml_path.string());
            auto robot_config = robot_builder(spot_urdf_string, spot_urdf_robot_builder_info);
            robot_description.slist = robot_config.Slist;
            robot_description.M = robot_config.M;
            robot_description.joint_states = Eigen::VectorXd::Zero(robot_config.joint_names.robot.size()); // just set to zero
	}catch (const std::exception &e)
        {
            std::cerr << "Error building robot: " << e.what() << std::endl;
        }

        std::cout << "FK: \n" << robot_description.M << std::endl;

        affordance_util::ScrewInfo screw_info;
    
        // Set the method to FROM_FK
        screw_info.from.method = affordance_util::PoseSpecificationMethod::FROM_FK;
    

        // Ask for +Z axis in final pose
        screw_info.from.axis_in_final_pose = affordance_util::axis_to_vec(affordance_util::Axis::X);
        std::cout << "Axis in final pose: " << screw_info.from.axis_in_final_pose.transpose() << std::endl;
    
        // Call function
        std::cout << "With post-transform as 90deg rotation about z-axis only:" << std::endl;
        {
        // Optionally apply a small post-transform (e.g., 10cm along X)
        screw_info.from.post_transform = Eigen::Isometry3d(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ())).matrix();

        affordance_util::ScrewInfo updated = get_affordance_info_from_fk(screw_info, robot_description);
    
        // Print results
        std::cout << "Location: " << updated.location.transpose() << std::endl;
        std::cout << "Axis: " << updated.axis.transpose() << std::endl;
	}

        // Now call again to test translation-only post-transform
        std::cout << "With post-transform as 10cm shift along x axis only:" << std::endl;
        {
        screw_info.from.post_transform = Eigen::Matrix4d::Identity();
        screw_info.from.post_transform(0, 3) = 0.1;
        affordance_util::ScrewInfo updated = get_affordance_info_from_fk(screw_info, robot_description);

        std::cout << "Location: " << updated.location.transpose() << std::endl;
        std::cout << "Axis: " << updated.axis.transpose() << std::endl;
        }
    }
    {
        std::cout << "\nTesting get_pose_from_fk\n" << std::endl;

        // We'll use Spot's robot description
        affordance_util::RobotDescription robot_description;
        try {
            std::filesystem::path spot_urdf_yaml_path = project_root / "test" / "cca_spot_urdf.yaml";
	    auto spot_urdf_robot_builder_info = extract_info_for_urdf_robot_builder(spot_urdf_yaml_path.string());
            auto robot_config = robot_builder(spot_urdf_string, spot_urdf_robot_builder_info);
            robot_description.slist = robot_config.Slist;
            robot_description.M = robot_config.M;
            robot_description.joint_states = Eigen::VectorXd::Zero(robot_config.joint_names.robot.size()); // just set to zero
	}catch (const std::exception &e)
        {
            std::cerr << "Error building robot: " << e.what() << std::endl;
        }

        std::cout << "FK: \n" << robot_description.M << std::endl;
    
        affordance_util::PoseFrom pose_from;
        pose_from.method = affordance_util::PoseSpecificationMethod::FROM_FK;
    
        // Post-transform: 90 degrees about Z axis
        pose_from.post_transform = Eigen::Matrix4d::Identity();
        pose_from.post_transform = Eigen::Isometry3d(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ())).matrix();
    
        Eigen::Matrix4d result = get_pose_from_fk(pose_from, robot_description);
    
        std::cout << "With post-transform as 90deg rotation about z-axis only:" << std::endl;
        std::cout << "Final Pose:\n" << result << std::endl;
    }




    return 0;
}
