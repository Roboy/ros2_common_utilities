#include "ros2_common_utilities/MotorConfig.hpp"

bool MotorConfig::readConfig(const string &filepath){
    if(!fileExists(filepath)) {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "motor config file does not exist, check your path");
        return false;
    }
    YAML::Node config = YAML::LoadFile(filepath);
    number_of_icebuses = config["number_of_icebuses"].as<int>();
    if(number_of_icebuses==0)
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"number_of_icebuses is zero! check your moto config file, if this is not on purpose");
    else
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"configuring %d icebuses",number_of_icebuses);
    for(int i=0;i<number_of_icebuses;i++){
        char str[20];
        sprintf(str,"icebus_%d",i);
        int number_of_motors = config[str]["number_of_motors"].as<int>();
        vector<int> bus_ids = config[str]["bus_ids"].as<vector<int>>();
        vector<int> motor_ids = config[str]["motor_ids"].as<vector<int>>();
        vector<int> motor_ids_global = config[str]["motor_ids_global"].as<vector<int>>();
        vector<string> muscleType = config[str]["muscle_type"].as<vector<string>>();
        vector<vector<float>> coeffs_force2displacement = config[str]["coeffs_force2displacement"].as<vector<vector<float>>>();
        vector<vector<float>> coeffs_displacement2force = config[str]["coeffs_displacement2force"].as<vector<vector<float>>>();
        if(motor_ids.size()>number_of_motors){
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"motor_ids of icebus %d does not match number_of_motors, check your motor config file, adjusting to number_of_motors parameter and continue",i);
            motor_ids.resize(number_of_motors);
        }
        if(bus_ids.size()>number_of_motors){
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"bus_ids of icebus %d does not match number_of_motors, check your motor config file, adjusting to number_of_motors parameter and continue",i);
            bus_ids.resize(number_of_motors);
        }
        if(motor_ids_global.size()>number_of_motors){
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"motor_ids_global of icebus %d does not match number_of_motors, check your motor config file, adjusting to number_of_motors parameter and continue",i);
            motor_ids_global.resize(number_of_motors);
        }
        if(coeffs_force2displacement.size()>number_of_motors){
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"coeffs_force2displacement of icebus %d does not match number_of_motors, check your motor config file, adjusting to number_of_motors parameter and continue",i);
            coeffs_force2displacement.resize(number_of_motors);
        }
        if(coeffs_displacement2force.size()>number_of_motors){
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"coeffs_displacement2force of icebus %d does not match number_of_motors, check your motor config file, adjusting to number_of_motors parameter and continue",i);
            coeffs_displacement2force.resize(number_of_motors);
        }
        for(int m=0;m<number_of_motors;m++){
            MotorPtr motor_ = MotorPtr(new Motor(i,bus_ids[m],motor_ids[m],motor_ids_global[m],muscleType[m],
                    coeffs_force2displacement[m],
                                       coeffs_displacement2force[m]));
            motor[motor_ids_global[m]] = motor_;
            icebus[i].push_back(motor_);
        }
        total_number_of_motors += number_of_motors;
    }

    number_of_myobuses = config["number_of_myobuses"].as<int>();
    if(number_of_myobuses==0)
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"number_of_myobuses is zero! check your moto config file, if this is not on purpose");
    else
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"configuring %d myobuses",number_of_myobuses);
    for(int i=0;i<number_of_myobuses;i++){
        char str[20];
        sprintf(str,"myobus_%d",i);
        int number_of_motors = config[str]["number_of_motors"].as<int>();
        vector<int> motor_ids = config[str]["motor_ids"].as<vector<int>>();
        vector<int> motor_ids_global = config[str]["motor_ids_global"].as<vector<int>>();
        vector<string> muscleType = config[str]["muscle_type"].as<vector<string>>();
        vector<vector<float>> coeffs_force2displacement = config[str]["coeffs_force2displacement"].as<vector<vector<float>>>();
        vector<vector<float>> coeffs_displacement2force = config[str]["coeffs_displacement2force"].as<vector<vector<float>>>();
        if(motor_ids.size()!=number_of_motors){
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"motor_ids of myobus %d does not match number_of_motors, check your motor config file",i);
            continue;
        }
        if(motor_ids_global.size()!=number_of_motors){
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"motor_ids_global of myobus %d does not match number_of_motors, check your motor config file",i);
            continue;
        }
        if(coeffs_force2displacement.size()!=number_of_motors){
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"coeffs_force2displacement of myobus %d does not match number_of_motors, check your motor config file",i);
            continue;
        }
        if(coeffs_displacement2force.size()!=number_of_motors){
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"coeffs_displacement2force of myobus %d does not match number_of_motors, check your motor config file",i);
            continue;
        }
        for(int m=0;m<number_of_motors;m++){
            MotorPtr motor_ = MotorPtr(new Motor(i,motor_ids[m],motor_ids[m],motor_ids_global[m],muscleType[m],
                                                 coeffs_force2displacement[m],
                                                 coeffs_displacement2force[m]));
            motor[motor_ids_global[m]] = motor_;
            myobus[i].push_back(motor_);
        }
        total_number_of_motors += number_of_motors;
    }

    vector<string> body_parts = config["body_part"]["name"].as<vector<string>>();
    vector<vector<int>> body_part_motor_ids_global = config["body_part"]["motor_ids_global"].as<vector<vector<int>>>();
    for(int i=0;i<body_parts.size();i++){
        body_part[i].reset(new BodyPart());
        body_part[i]->name = body_parts[i];
        for(int j=0;j<body_part_motor_ids_global[i].size();j++){
            if(motor.find(body_part_motor_ids_global[i][j]) != motor.end())
              body_part[i]->motor_ids_global.push_back(motor[body_part_motor_ids_global[i][j]]);
            else
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"the motor %d assigned to body_part %s does not exist, check your motor config file", body_part_motor_ids_global[i][j], body_part[i]->name.c_str());
        }
    }
    return true;
}

bool MotorConfig::writeConfig(const string &filepath){
//    std::ofstream fout(filepath);
//    if (!fout.is_open()) {
//        ROS_WARN_STREAM("Could not write config " << filepath);
//        return false;
//    }
//    YAML::Node config;
//    for(int fpga = 0; fpga<NUMBER_OF_FPGAS; fpga++) {
//        char fpgastr[200];
//        sprintf(fpgastr,"fpga%d",fpga);
//        int motor = 0;
//        for (auto const &coefficients : coeffs_displacement2force[fpga]) {
//            stringstream str;
//            str << "[";
//            for (uint i = 0; i <= coefficients.size(); i++) {
//                if (i < coefficients.size())
//                    str << "0,";
//                else
//                    str << "0]";
//            }
//            YAML::Node node = YAML::Load(str.str());
//            // first number is the sensor id
//            node[0] = motor;
//            for (int i = 0; i < coefficients.size(); i++) {
//                node[i + 1] = coefficients[i];
//            }
//            config[fpgastr]["motor_polynomial_parameters_displacement2force"].push_back(node);
//            motor++;
////        ROS_DEBUG_STREAM(coefficients.first << "\t" << coeffs_displacement2force[coefficients.first][0]<< "\t" << coeffs_displacement2force[coefficients.first][1]
////                                           << "\t" << coeffs_displacement2force[coefficients.first][2]<< "\t" << coeffs_displacement2force[coefficients.first][3]
////                                           << "\t" << coeffs_displacement2force[coefficients.first][4]);
//        }
//        motor = 0;
//        for (auto const &coefficients : coeffs_force2displacement[fpga]) {
//            stringstream str;
//            str << "[";
//            for (uint i = 0; i <= coefficients.size(); i++) {
//                if (i < coefficients.size())
//                    str << "0,";
//                else
//                    str << "0]";
//            }
//            YAML::Node node = YAML::Load(str.str());
//            // first number is the sensor id
//            node[0] = motor;
//            for (int i = 0; i < coefficients.size(); i++) {
//                node[i + 1] = coefficients[i];
//            }
//            config[fpgastr]["motor_polynomial_parameters_force2displacement"].push_back(node);
//            motor++;
////        ROS_DEBUG_STREAM(coefficients.first << "\t" << coeffs_force2displacement[coefficients.first][0]<< "\t" << coeffs_force2displacement[coefficients.first][1]
////                                           << "\t" << coeffs_force2displacement[coefficients.first][2]<< "\t" << coeffs_force2displacement[coefficients.first][3]
////                                           << "\t" << coeffs_force2displacement[coefficients.first][4]);
//        }
//    }
//    fout << config;
    return true;
}

bool MotorConfig::fileExists(const string &filepath){
    struct stat buffer;
    return (stat (filepath.c_str(), &buffer) == 0);
}

double MotorConfig::displacement2force(double displacement, int motor_id_global){
    double force = 0;
    for (uint i = 0; i < motor[motor_id_global]->coeffs_displacement2force.size(); i++) {
        force += motor[motor_id_global]->coeffs_displacement2force[i] * pow(displacement, (double) i);
    }
    return force;
}

double MotorConfig::force2displacement(double force, int motor_id_global){
    double displacement = 0;
    for (uint i = 0; i < motor[motor_id_global]->coeffs_force2displacement.size(); i++) {
        displacement += motor[motor_id_global]->coeffs_force2displacement[i] * pow(force, (double) i);
    }
    return displacement;
}
