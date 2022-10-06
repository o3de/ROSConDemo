/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "PickingStructs.h"
#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <AzCore/std/containers/vector.h>

namespace AppleKraken
{
    //! A class which provides ground truth for apples. For now it is only for debug.
    class AppleDetectionGroundTruth
    {
    public:
        AppleDetectionGroundTruth(const AZStd::string& rosNamespace, const AZStd::string& namespacedFrameId);
        void UpdateGroundTruth(const AZStd::vector<PickAppleTask>& apples);
        void Publish();

    private:
        // TODO - templates would improve it here (code is similar)
        vision_msgs::msg::Detection3D Construct3DDetection(const PickAppleTask& apple);
        void ConstructROS2Detection3DMessage(const AZStd::vector<PickAppleTask>& apples);

        vision_msgs::msg::Detection2D Construct2DDetection(const PickAppleTask& apple);
        void ConstructROS2Detection2DMessage(const AZStd::vector<PickAppleTask>& apples);

        AZStd::string m_frameId; //!< includes namespace
        std::shared_ptr<rclcpp::Publisher<vision_msgs::msg::Detection3DArray>> m_detection3DPublisher;
        std::shared_ptr<rclcpp::Publisher<vision_msgs::msg::Detection2DArray>> m_detection2DPublisher;
        vision_msgs::msg::Detection3DArray m_appleDetections3DMessage;
        vision_msgs::msg::Detection2DArray m_appleDetections2DMessage;
    };
} // namespace AppleKraken
