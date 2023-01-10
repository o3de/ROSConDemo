/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/

#pragma once

#include <AzCore/Component/Component.h>
#include <RobotHarvestingSample/RobotHarvestingSampleBus.h>
#include <nav_msgs/srv/get_plan.hpp>
#include <rclcpp/rclcpp.hpp>

namespace RobotHarvestingSample
{
    using GetPlanRequestPtr = std::shared_ptr<nav_msgs::srv::GetPlan::Request>;
    using GetPlanResponsePtr = std::shared_ptr<nav_msgs::srv::GetPlan::Response>;

    class RobotHarvestingSampleSystemComponent
        : public AZ::Component
        , protected RobotHarvestingSampleRequestBus::Handler
    {
    public:
        AZ_COMPONENT(RobotHarvestingSampleSystemComponent, "{194FFE4C-CA95-400E-BCA2-CB5083ABEC5F}");

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        RobotHarvestingSampleSystemComponent();
        ~RobotHarvestingSampleSystemComponent();

    protected:
        void Activate() override;
        void Deactivate() override;

    private:
        void ProcessGetPlanServiceCall(const GetPlanRequestPtr req, GetPlanResponsePtr resp);
        void ReloadLevel() override;
        
        const AZStd::string m_planTopic = "get_gathering_plan";
        rclcpp::Service<nav_msgs::srv::GetPlan>::SharedPtr m_pathPlanService;
    };
} // namespace RobotHarvestingSample
