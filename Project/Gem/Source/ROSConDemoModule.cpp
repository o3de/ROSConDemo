
#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/Module/Module.h>

#include "ROSConDemoSystemComponent.h"

namespace ROSConDemo
{
    class ROSConDemoModule
        : public AZ::Module
    {
    public:
        AZ_RTTI(ROSConDemoModule, "{E38575E4-7D2F-4617-B938-416E8C1C07B4}", AZ::Module);
        AZ_CLASS_ALLOCATOR(ROSConDemoModule, AZ::SystemAllocator, 0);

        ROSConDemoModule()
            : AZ::Module()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            m_descriptors.insert(m_descriptors.end(), {
                ROSConDemoSystemComponent::CreateDescriptor(),
            });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<ROSConDemoSystemComponent>(),
            };
        }
    };
}// namespace ROSConDemo

AZ_DECLARE_MODULE_CLASS(Gem_ROSConDemo, ROSConDemo::ROSConDemoModule)
