/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/

#include "ROSConDemoSystemComponent.h"
#include "ApplePicker/GatheringRowRequests.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/Utilities/ROS2Conversions.h>

namespace ROSConDemo
{
    void ROSConDemoSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROSConDemoSystemComponent, AZ::Component>()->Version(0);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ROSConDemoSystemComponent>("ROSConDemo", "[Description of functionality provided by this System Component]")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("System"))
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true);
            }
        }
    }

    void ROSConDemoSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC("ROSConDemoService"));
    }

    void ROSConDemoSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC("ROSConDemoService"));
    }

    void ROSConDemoSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void ROSConDemoSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    ROSConDemoSystemComponent::ROSConDemoSystemComponent()
    {
        if (ROSConDemoInterface::Get() == nullptr)
        {
            ROSConDemoInterface::Register(this);
        }
    }

    ROSConDemoSystemComponent::~ROSConDemoSystemComponent()
    {
        if (ROSConDemoInterface::Get() == this)
        {
            ROSConDemoInterface::Unregister(this);
        }
    }

    void ROSConDemoSystemComponent::ProcessGetPlanServiceCall(const GetPlanRequestPtr req, GetPlanResponsePtr resp)
    {
        AppleKraken::GatheringPoses results;
        AppleKraken::GatheringRowRequestBus::BroadcastResult(results, &AppleKraken::GatheringRowRequests::GetGatheringPoses);
        // TODO - get closest to startTransform
        // auto startTransform = ROS2::ROS2Conversions::FromROS2Pose(req->start.pose);

        for (auto result : results)
        {
            geometry_msgs::msg::PoseStamped stampedPose; // TODO - fill in header
            stampedPose.pose = ROS2::ROS2Conversions::ToROS2Pose(result);
            resp->plan.poses.push_back(stampedPose);
        }
    }

    void ROSConDemoSystemComponent::Activate()
    {   // TODO - the service should probably only be created in Game Mode
        ROSConDemoRequestBus::Handler::BusConnect();
        auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();
        m_pathPlanService = ros2Node->create_service<nav_msgs::srv::GetPlan>(
            m_planTopic.c_str(),
            [this](const GetPlanRequestPtr request, GetPlanResponsePtr response)
            {
                this->ProcessGetPlanServiceCall(request, response);
            });
    }

    void ROSConDemoSystemComponent::Deactivate()
    {
        m_pathPlanService.reset();
        ROSConDemoRequestBus::Handler::BusDisconnect();
    }
} // namespace ROSConDemo
