#pragma once

#include "ControlAllocation.hpp"
#include <lib/eigen/Eigen/Dense>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>


class ControlAllocationNeural: public ControlAllocation
{
public:
	ControlAllocationNeural();
	virtual ~ControlAllocationNeural() = default;

	void allocate() override;
    Eigen::VectorXf scale_wrench(Eigen::VectorXf wrench);
    Eigen::VectorXf forwardPath(Eigen::VectorXf input);

private:
    Eigen::VectorXf _bias_allocation_net_layer_1;
    Eigen::MatrixXf _weight_allocation_net_layer_1;
    Eigen::VectorXf _bias_allocation_net_layer_2;
    Eigen::MatrixXf _weight_allocation_net_layer_2;
};
