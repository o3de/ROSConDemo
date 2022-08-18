
#pragma once

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace ROSConDemo
{
    class ROSConDemoRequests
    {
    public:
        AZ_RTTI(ROSConDemoRequests, "{2D614DC7-0E0B-4EC9-B0EB-8F2ED827310A}");
        virtual ~ROSConDemoRequests() = default;
        // Put your public methods here
    };

    class ROSConDemoBusTraits
        : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using ROSConDemoRequestBus = AZ::EBus<ROSConDemoRequests, ROSConDemoBusTraits>;
    using ROSConDemoInterface = AZ::Interface<ROSConDemoRequests>;

} // namespace ROSConDemo
