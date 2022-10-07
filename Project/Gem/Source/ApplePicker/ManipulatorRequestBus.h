/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/EBus/EBus.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/RTTI/BehaviorContext.h>

namespace AppleKraken
{
    class ManipulatorRequest : public AZ::EBusTraits
    {
    public:
        static const AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static const AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;
        // Messages are addressed by EntityId.
        using BusIdType = AZ::EntityId;
        virtual void PickApple(const AZ::Vector3 position) = 0;
        virtual AZ::Vector3 GetPosition() = 0;
        virtual void Retrieve() = 0;
        virtual int GetStatus() = 0;
    };

    using ManipulatorRequestBus = AZ::EBus<ManipulatorRequest>;

    //! EventHanlder with AZ_EBUS_BEHAVIOR_BINDER to integrate with LUA/Canvas
    class ManipulatorRequestHandler
        : public ManipulatorRequestBus::Handler
        , public AZ::BehaviorEBusHandler
    {
    public:
        AZ_EBUS_BEHAVIOR_BINDER(
            ManipulatorRequestHandler,
            "{30CE1753-DEDE-4D83-8C7C-F5F2BBD12DE8}",
            AZ::SystemAllocator,
            PickApple,
            GetPosition,
            Retrieve,
            GetStatus);

        virtual void PickApple(const AZ::Vector3 position) override;

        virtual AZ::Vector3 GetPosition() override;

        virtual void Retrieve() override;

        virtual int GetStatus() override;

        static void Reflect(AZ::ReflectContext* context);
    };
} // namespace AppleKraken