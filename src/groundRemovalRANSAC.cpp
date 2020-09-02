//
// Created by xcy on 2020/5/30.
//

#include "groundRemovalRANSAC.h"



/*带左右双地面的会导致右边的地面往上翘，前面地面去除不干净，需要切除更多的数目*/
void ransacDriver::groundRemoveSided(sensor_msgs::PointCloud &PCLin) {

    PCLforOutput = PCLin;

    /*Open Channels*/
    PCLforOutput.channels.clear();
    PCLforOutput.channels.resize(2);
    PCLforOutput.channels[CANDIDATE].name = "CANDIDATE";
    PCLforOutput.channels[CANDIDATE].values.resize(PCLforOutput.points.size(), 0);
    PCLforOutput.channels[INLINER].name = "INLINER";
    PCLforOutput.channels[INLINER].values.resize(PCLforOutput.points.size(), 0);

    /*only use near ground pts*/
    double CandidateNumLeft = 0.0;
    double CandidateNumRight = 0.0;
    vector<int> CandidateIndexLeft;
    vector<int> CandidateIndexRight;
    for(int i = 0; i < PCLforOutput.points.size(); i++){
        if(PCLforOutput.points[i].z >= groundZMin)
            if(PCLforOutput.points[i].z <= groundZMax){
                if(PCLforOutput.points[i].y >= 0){
                    PCLforOutput.channels[CANDIDATE].values[i] = 1;
                    CandidateIndexLeft.push_back(i);
                    CandidateNumLeft ++;
                }else{
                    PCLforOutput.channels[CANDIDATE].values[i] = 2;
                    CandidateIndexRight.push_back(i);
                    CandidateNumRight ++;
                }
            }
    }

    double iterationNum = ceil(log(1 - confidence)/log(1 - pow(inlierRatio, sampleNum)));

    double bestALeft, bestBLeft, bestCLeft, bestNumInlinerLeft = -INFINITY;
    //ROS_INFO("it num: %f ",iterationNum);
    for(int itn = 0; itn < iterationNum; itn++){
        /*RANdom SAmple and Fittig*/
        double a = 0.0;
        double b = 0.0;
        double c = -1.0;

        ceres::Problem problem;
        for (int i = 0; i < sampleNum ; ++i) {
            int lucky = CandidateIndexLeft[int(rand() / double(RAND_MAX) * CandidateIndexLeft.size())];
            //ROS_INFO("%f,%f,%f",PCLforOutput.points[lucky].x,PCLforOutput.points[lucky].y,PCLforOutput.points[lucky].z);
            problem.AddResidualBlock(
                    new ceres::AutoDiffCostFunction<planeResidual, 1, 1, 1, 1>(//out+1
                            new planeResidual(PCLforOutput.points[lucky].x, PCLforOutput.points[lucky].y, PCLforOutput.points[lucky].z)),
                    //inverse normalization and transfer from deg. to rad.
                    NULL,
                    &a, &b, &c);
        }
        //solve the problem
        ceres::Solver::Options options;
        options.max_num_iterations = 100;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = false;
        ceres::Solver::Summary summary;
        Solve(options, &problem, &summary);
        //std::cout << "Before   a: " << a << " b: " << b << " c: " << c <<"\n";

        /*Compute the distance and Num of Inliner*/
        int inlinerSum = 0;
        for(int i = 0; i < PCLforOutput.points.size(); i++){
            if(PCLforOutput.channels[CANDIDATE].values[i] != 1){
                continue;
            }
            double distance = abs(a * PCLforOutput.points[i].x + b * PCLforOutput.points[i].y+ c * PCLforOutput.points[i].z -1);
            distance = distance / sqrt (a*a + b*b + c*c);
            if(distance < inlinerThreshold){
                PCLforOutput.channels[INLINER].values[i] = 1;
                inlinerSum++;
            }
        }
        //ROS_INFO("current ratio: %f", double(inlinerSum)/CandidateNum);

        /*See if the best*/
        if(inlinerSum > bestNumInlinerLeft){
            bestALeft = a;
            bestBLeft = b;
            bestCLeft = c;
            bestNumInlinerLeft = inlinerSum;
        }
        for(int i = 0; i < CandidateIndexLeft.size(); i++){
            PCLforOutput.channels[INLINER].values[CandidateIndexLeft[i]] = 0;
        }
        /*End Condition*/
        double currentBestRatio = bestNumInlinerLeft / CandidateNumLeft;
        if(currentBestRatio > ratioCondition){
            //ROS_INFO("End earlier with ratio Condition: %f", currentBestRatio);
            break;
        }
    }


    double bestARight, bestBRight, bestCRight, bestNumInlinerRight = -INFINITY;
    //ROS_INFO("it num: %f ",iterationNum);
    for(int itn = 0; itn < iterationNum; itn++){
        /*RANdom SAmple and Fittig*/
        double a = 0.0;
        double b = 0.0;
        double c = -1.0;

        ceres::Problem problem;
        for (int i = 0; i < sampleNum ; ++i) {
            int lucky = CandidateIndexRight[int(rand() / double(RAND_MAX) * CandidateIndexRight.size())];
            //ROS_INFO("%f,%f,%f",PCLforOutput.points[lucky].x,PCLforOutput.points[lucky].y,PCLforOutput.points[lucky].z);
            problem.AddResidualBlock(
                    new ceres::AutoDiffCostFunction<planeResidual, 1, 1, 1, 1>(//out+1
                            new planeResidual(PCLforOutput.points[lucky].x, PCLforOutput.points[lucky].y, PCLforOutput.points[lucky].z)),
                    //inverse normalization and transfer from deg. to rad.
                    NULL,
                    &a, &b, &c);
        }
        //solve the problem
        ceres::Solver::Options options;
        options.max_num_iterations = 100;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = false;
        ceres::Solver::Summary summary;
        Solve(options, &problem, &summary);
        //std::cout << "Before   a: " << a << " b: " << b << " c: " << c <<"\n";

        /*Compute the distance and Num of Inliner*/
        int inlinerSum = 0;
        for(int i = 0; i < PCLforOutput.points.size(); i++){
            if(PCLforOutput.channels[CANDIDATE].values[i] != 2){
                continue;
            }
            double distance = abs(a * PCLforOutput.points[i].x + b * PCLforOutput.points[i].y+ c * PCLforOutput.points[i].z -1);
            distance = distance / sqrt (a*a + b*b + c*c);
            if(distance < inlinerThreshold){
                PCLforOutput.channels[INLINER].values[i] = 1;
                inlinerSum++;
            }
        }
        //ROS_INFO("current ratio: %f", double(inlinerSum)/CandidateNum);

        /*See if the best*/
        if(inlinerSum > bestNumInlinerRight){
            bestARight = a;
            bestBRight = b;
            bestCRight = c;
            bestNumInlinerRight = inlinerSum;
        }
        for(int i = 0; i < CandidateIndexRight.size(); i++){
            PCLforOutput.channels[INLINER].values[CandidateIndexRight[i]] = 0;
        }
        /*End Condition*/
        double currentBestRatio = bestNumInlinerRight / CandidateNumLeft;
        if(currentBestRatio > ratioCondition){
            //ROS_INFO("End earlier with ratio Condition: %f", currentBestRatio);
            break;
        }
    }



    for(int i = 0; i < PCLforOutput.channels[INLINER].values.size(); i++){
        PCLforOutput.channels[INLINER].values[i] = 0;
    }
    for(int i = 0; i < PCLforOutput.points.size(); i++){
        if(PCLforOutput.points[i].y >= 0){
            double function = bestALeft * PCLforOutput.points[i].x + bestBLeft * PCLforOutput.points[i].y + bestCLeft * PCLforOutput.points[i].z - 1;
            if(function >= 0){
                PCLforOutput.channels[INLINER].values[i] = 1;
            }else{
                function = abs(function) / sqrt (bestALeft * bestALeft + bestBLeft * bestBLeft + bestCLeft * bestCLeft);
                if(function < upperBorder){
                    PCLforOutput.channels[INLINER].values[i] = 1;
                }else{
                    PCLforOutput.channels[INLINER].values[i] = 0;
                }
            }
        }

        if(PCLforOutput.points[i].y < 0){
            double function = bestARight * PCLforOutput.points[i].x + bestBRight * PCLforOutput.points[i].y + bestCRight * PCLforOutput.points[i].z - 1;
            if(function >= 0){
                PCLforOutput.channels[INLINER].values[i] = 1;
            }else{
                function = abs(function) / sqrt (bestARight * bestARight + bestBRight * bestBRight + bestCRight * bestCRight);
                if(function < upperBorder){
                    PCLforOutput.channels[INLINER].values[i] = 1;
                }else{
                    PCLforOutput.channels[INLINER].values[i] = 0;
                }
            }
        }
    }

    //PCLforOutput.channels[INLINER] = bestCandidates;
    //ROS_INFO("Final LEFT Plane is %f x + %f y + %f z - 1 = 0 with ratio: %f", bestALeft, bestBLeft, bestCLeft, bestNumInlinerLeft / CandidateNumLeft);
    //ROS_INFO("Final RIGHT Plane is %f x + %f y + %f z - 1 = 0 with ratio: %f", bestARight, bestBRight, bestCRight, bestNumInlinerRight / CandidateNumRight);
}

void ransacDriver::groundRemove(sensor_msgs::PointCloud &PCLin) {

    PCLforOutput = PCLin;

    /*Open Channels*/
    PCLforOutput.channels.clear();
    PCLforOutput.channels.resize(2);
    PCLforOutput.channels[CANDIDATE].name = "CANDIDATE";
    PCLforOutput.channels[CANDIDATE].values.resize(PCLforOutput.points.size(), 0);
    PCLforOutput.channels[INLINER].name = "INLINER";
    PCLforOutput.channels[INLINER].values.resize(PCLforOutput.points.size(), 0);

    /*only use near ground pts*/
    double CandidateNum = 0.0;

    vector<int> CandidateIndex;

    for(int i = 0; i < PCLforOutput.points.size(); i++){
        if(PCLforOutput.points[i].z >= groundZMin)
            if(PCLforOutput.points[i].z <= groundZMax){
                PCLforOutput.channels[CANDIDATE].values[i] = 1;
                CandidateIndex.push_back(i);
                CandidateNum ++;
            }
    }

    double iterationNum = ceil(log(1 - confidence)/log(1 - pow(inlierRatio, sampleNum)));

    double bestA, bestB, bestC, bestNumInliner = -INFINITY;
    //ROS_INFO("it num: %f ",iterationNum);
    for(int itn = 0; itn < iterationNum; itn++){
        /*RANdom SAmple and Fittig*/
        double a = 0.0;
        double b = 0.0;
        double c = -1.0;

        ceres::Problem problem;
        for (int i = 0; i < sampleNum ; ++i) {
            int lucky = CandidateIndex[int(rand() / double(RAND_MAX) * CandidateIndex.size())];
            //ROS_INFO("%f,%f,%f",PCLforOutput.points[lucky].x,PCLforOutput.points[lucky].y,PCLforOutput.points[lucky].z);
            problem.AddResidualBlock(
                    new ceres::AutoDiffCostFunction<planeResidual, 1, 1, 1, 1>(//out+1
                            new planeResidual(PCLforOutput.points[lucky].x, PCLforOutput.points[lucky].y, PCLforOutput.points[lucky].z)),
                    //inverse normalization and transfer from deg. to rad.
                    NULL,
                    &a, &b, &c);
        }
        //solve the problem
        ceres::Solver::Options options;
        options.max_num_iterations = 100;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = false;
        ceres::Solver::Summary summary;
        Solve(options, &problem, &summary);
        //std::cout << "Before   a: " << a << " b: " << b << " c: " << c <<"\n";

        /*Compute the distance and Num of Inliner*/
        int inlinerSum = 0;
        for(int i = 0; i < PCLforOutput.points.size(); i++){
            if(PCLforOutput.channels[CANDIDATE].values[i] != 1){
                continue;
            }
            double distance = abs(a * PCLforOutput.points[i].x + b * PCLforOutput.points[i].y+ c * PCLforOutput.points[i].z -1);
            distance = distance / sqrt (a*a + b*b + c*c);
            if(distance < inlinerThreshold){
                PCLforOutput.channels[INLINER].values[i] = 1;
                inlinerSum++;
            }
        }
        //ROS_INFO("current ratio: %f", double(inlinerSum)/CandidateNum);

        /*See if the best*/
        if(inlinerSum > bestNumInliner){
            bestA = a;
            bestB = b;
            bestC = c;
            bestNumInliner = inlinerSum;
        }
        for(int i = 0; i < CandidateIndex.size(); i++){
            PCLforOutput.channels[INLINER].values[CandidateIndex[i]] = 0;
        }
        /*End Condition*/
        double currentBestRatio = bestNumInliner / CandidateNum;
        if(currentBestRatio > ratioCondition){
            //ROS_INFO("End earlier with ratio Condition: %f", currentBestRatio);
            break;
        }
    }


    for(int i = 0; i < PCLforOutput.channels[INLINER].values.size(); i++){
        PCLforOutput.channels[INLINER].values[i] = 0;
    }
    for(int i = 0; i < PCLforOutput.points.size(); i++){
        double function = bestA * PCLforOutput.points[i].x + bestB * PCLforOutput.points[i].y + bestC * PCLforOutput.points[i].z - 1;
        if(function >= 0){
            PCLforOutput.channels[INLINER].values[i] = 1;
        }else{
            function = abs(function) / sqrt (bestA * bestA + bestB * bestB + bestC * bestC);
            if(function < upperBorder){
                PCLforOutput.channels[INLINER].values[i] = 1;
            }else{
                PCLforOutput.channels[INLINER].values[i] = 0;
            }
        }
}

    //PCLforOutput.channels[INLINER] = bestCandidates;
    //ROS_INFO("Final Plane is %f x + %f y + %f z - 1 = 0 with ratio: %f", bestA, bestB, bestC, bestNumInliner / CandidateNum);

}
