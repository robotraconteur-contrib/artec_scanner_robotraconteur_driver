#include "experimental__artec_scanner.h"
#include "experimental__artec_scanner_stubskel.h"
#include <artec/sdk/base/IFrameMesh.h>
#include <com__robotraconteur__geometry__shapes.h>

#define RR_CALL_ARTEC(cmd, user_err_msg) \
    { \
    artec::sdk::base::ErrorCode __rr_call_artec_res = cmd; \
    if (__rr_call_artec_res != artec::sdk::base::ErrorCode_OK) { \
        ROBOTRACONTEUR_LOG_ERROR_COMPONENTNAME(RR::RobotRaconteurNode::weak_sp(), ThirdParty,\
            "artec_scanner", "", 0, artec_scanner_robotraconteur_driver::ArtecErrorCodeLogMessage(__rr_call_artec_res) << user_err_msg) \
        artec_scanner_robotraconteur_driver::ThrowArtecErrorCode(__rr_call_artec_res, user_err_msg); \
    } \
    } 

#define RR_ARTEC_LOG_ERROR(msg) \
    ROBOTRACONTEUR_LOG_ERROR_COMPONENTNAME(RR::RobotRaconteurNode::weak_sp(), ThirdParty, \
    "artec_scanner", "", 0, msg) \

#define RR_ARTEC_LOG_INFO(msg) \
    ROBOTRACONTEUR_LOG_INFO_COMPONENTNAME(RR::RobotRaconteurNode::weak_sp(), ThirdParty, \
    "artec_scanner", "", 0, msg) \

namespace artec_scanner_robotraconteur_driver
{
    com::robotraconteur::geometry::shapes::MeshPtr ConvertArtecMeshToRR(artec::sdk::base::IFrameMesh* mesh);

    void ThrowArtecErrorCode(artec::sdk::base::ErrorCode ec, const std::string& user_msg);

    void ArtecErrorCodeMessage(artec::sdk::base::ErrorCode ec, std::string& msg, std::string& suberr);
    
    std::string ArtecErrorCodeLogMessage(artec::sdk::base::ErrorCode ec);
}