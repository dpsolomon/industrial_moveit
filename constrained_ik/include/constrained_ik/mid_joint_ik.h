/*
 * mid_joint_ik.h
 *
 *  Created on: Sep 23, 2013
 *      Author: dsolomon
 */
/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2013, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef MID_JOINT_IK_H_
#define MID_JOINT_IK_H_

#include "basic_ik.h"

namespace constrained_ik{
namespace mid_joint_ik {


class MidJoint_IK: public basic_ik::Basic_IK {
public:
    MidJoint_IK();
    virtual ~MidJoint_IK() {};

    void calcInvKin(const Eigen::Affine3d &goal, const Eigen::VectorXd &joint_seed, Eigen::VectorXd &joint_angles);

    void init(const basic_kin::BasicKin &kin);

protected:

    Eigen::VectorXd mid_joint_;	// middle of joint range

    //TODO document
    void augmentJ(Eigen::MatrixXd &J);

    //TODO document
    void augmentErr(Eigen::VectorXd &err);
};

} /* namespace mid_joint_ik */
} /* namespace constrained_ik */
#endif /* MID_JOINT_IK_H_ */
