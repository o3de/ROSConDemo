/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "AppleDetectionGroundTruth.h"
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2Bus.h>
#include "ROS2/Utilities/ROS2Conversions.h"
#include <ROS2/Utilities/ROS2Names.h>
#include <rclcpp/qos.hpp>

using namespace ROS2;

namespace AppleKraken
{
    AppleDetectionGroundTruth::AppleDetectionGroundTruth(const AZStd::string& rosNamespace, const AZStd::string& namespacedFrameId)
        : m_frameId(namespacedFrameId)
    {
        auto defaultQoS = rclcpp::QoS(10);
        auto ros2Node = ROS2Interface::Get()->GetNode();

        m_appleDetections3DMessage.header.frame_id = m_frameId.c_str();
        AZStd::string full3DTopic = ROS2Names::GetNamespacedName(rosNamespace, "ground_truth_3D_detection");
        m_detection3DPublisher = ros2Node->create_publisher<vision_msgs::msg::Detection3DArray>(full3DTopic.data(), defaultQoS);

        m_appleDetections2DMessage.header.frame_id = m_frameId.c_str();
        AZStd::string full2DTopic = ROS2Names::GetNamespacedName(rosNamespace, "ground_truth_2D_detection");
        m_detection2DPublisher = ros2Node->create_publisher<vision_msgs::msg::Detection2DArray>(full2DTopic.data(), defaultQoS);
    }

    void AppleDetectionGroundTruth::UpdateGroundTruth(const AZStd::vector<PickAppleTask>& apples)
    {
        ConstructROS2Detection2DMessage(apples);
        ConstructROS2Detection3DMessage(apples);
    }

    void AppleDetectionGroundTruth::Publish()
    {
        auto timestamp = ROS2Interface::Get()->GetROSTimestamp();
        m_appleDetections2DMessage.header.stamp = timestamp;
        for (auto& detection : m_appleDetections2DMessage.detections)
        {
            detection.header.stamp = timestamp;
        }
        m_detection2DPublisher->publish(m_appleDetections2DMessage);

        m_appleDetections3DMessage.header.stamp = timestamp;
        for (auto& detection : m_appleDetections3DMessage.detections)
        {
            detection.header.stamp = timestamp;
        }
        m_detection3DPublisher->publish(m_appleDetections3DMessage);
    }

    vision_msgs::msg::Detection3D AppleDetectionGroundTruth::Construct3DDetection(const PickAppleTask& apple)
    {
        vision_msgs::msg::Detection3D detection;
        detection.header.frame_id = m_frameId.c_str();
        detection.id = "Apple"; // duh
        auto middleVector = ROS2Conversions::ToROS2Vector3(apple.m_middle);
        detection.bbox.center.position.x = middleVector.x;
        detection.bbox.center.position.y = middleVector.y;
        detection.bbox.center.position.z = middleVector.z;
        detection.bbox.size = ROS2Conversions::ToROS2Vector3(apple.m_appleBoundingBox.GetExtents());
        return detection;
    }

    void AppleDetectionGroundTruth::ConstructROS2Detection3DMessage(const AZStd::vector<PickAppleTask>& apples)
    {
        m_appleDetections3DMessage.detections.clear();
        for (const auto& apple : apples)
        {
            m_appleDetections3DMessage.detections.push_back(Construct3DDetection(apple));
        }
    }

    vision_msgs::msg::Detection2D AppleDetectionGroundTruth::Construct2DDetection(const PickAppleTask& apple)
    {
        vision_msgs::msg::Detection2D detection;
        detection.header.frame_id = m_frameId.c_str();
        detection.id = "Apple";

        // TODO - warning, these APIs changed between Humble and Galactic!
        // In Galactic, center has no position, just x and y. In Humble, it has position.
        // detection.bbox.center.position.x = apple.m_middle.GetX();
        // detection.bbox.center.position.y = apple.m_middle.GetY();
        detection.bbox.size_x = apple.m_appleBoundingBox.GetExtents().GetX();
        detection.bbox.size_y = apple.m_appleBoundingBox.GetExtents().GetY();
        return detection;
    }

    void AppleDetectionGroundTruth::ConstructROS2Detection2DMessage(const AZStd::vector<PickAppleTask>& apples)
    {
        m_appleDetections2DMessage.detections.clear();
        for (const auto& apple : apples)
        {
            m_appleDetections2DMessage.detections.push_back(Construct2DDetection(apple));
        }
    }
} // namespace AppleKraken
