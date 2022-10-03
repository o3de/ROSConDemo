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


namespace ROSConDemo
{
    class ManipulatorRequest : public AZ::EBusTraits
    {
    public:
        static const AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static const AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;
        // Messages are addressed by EntityId.
        using BusIdType = AZ::EntityId;
        virtual void ManipulatorSetPosition(const AZ::Vector3 position) = 0;
        virtual AZ::Vector3 ManipulatorReportError() = 0;
        virtual int ManipulatorGetStatus() =0;
        virtual void ManipulatorRetract() = 0;

    };

    using ManipulatorRequestBus = AZ::EBus<ManipulatorRequest>;

    //! This simple handler can be used for using with LUA
    class ManipulatorRequestHandler
            : public ManipulatorRequestBus::Handler, public AZ::BehaviorEBusHandler{
    public:
        AZ_EBUS_BEHAVIOR_BINDER(ManipulatorRequestHandler, "{30CE1753-DEDE-4D83-8C7C-F5F2BBD12DE8}",
                                AZ::SystemAllocator, ManipulatorSetPosition, ManipulatorReportError,
                                ManipulatorGetStatus, ManipulatorRetract);

        virtual void ManipulatorSetPosition(const AZ::Vector3 position) override;

        virtual AZ::Vector3 ManipulatorReportError() override;

        virtual int ManipulatorGetStatus() override;

        virtual void ManipulatorRetract() override;

        static void Reflect(AZ::ReflectContext *context);
    };
}