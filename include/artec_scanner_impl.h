#include "experimental__artec_scanner.h"
#include "experimental__artec_scanner_stubskel.h"
#include <artec/sdk/capturing/IScanner.h>

namespace artec_scanner_robotraconteur_driver
{
    class ArtecScannerImpl : public experimental::artec_scanner::ArtecScanner_default_impl, 
        public RR_ENABLE_SHARED_FROM_THIS<ArtecScannerImpl>
    {

        private:
            artec::sdk::capturing::IScanner* scanner = nullptr;
            artec::sdk::capturing::IFrameProcessor* processor = nullptr;

        public:

            void Init(artec::sdk::capturing::IScanner* scanner);

            RR_INTRUSIVE_PTR<com::robotraconteur::geometry::shapes::Mesh > capture(RobotRaconteur::rr_bool with_texture) override;

            virtual ~ArtecScannerImpl();
    };
}