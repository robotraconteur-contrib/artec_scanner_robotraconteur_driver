#include "experimental__artec_scanner.h"
#include "experimental__artec_scanner_stubskel.h"
#include <artec/sdk/base/IFrameMesh.h>
#include <artec/sdk/base/ICompositeMesh.h>
#include <artec/sdk/base/TRef.h>
#include <artec/sdk/base/AlgorithmWorkset.h>
#include <artec/sdk/base/IModel.h>
#include <artec/sdk/base/ICancellationTokenSource.h>
#include <com__robotraconteur__geometry__shapes.h>

#pragma once

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

#define RR_ARTEC_LOG_WARNING(msg) \
    ROBOTRACONTEUR_LOG_WARNING_COMPONENTNAME(RR::RobotRaconteurNode::weak_sp(), ThirdParty, \
    "artec_scanner", "", 0, msg) \

#define RR_ARTEC_LOG_INFO(msg) \
    ROBOTRACONTEUR_LOG_INFO_COMPONENTNAME(RR::RobotRaconteurNode::weak_sp(), ThirdParty, \
    "artec_scanner", "", 0, msg) \

namespace artec_scanner_robotraconteur_driver
{
    com::robotraconteur::geometry::shapes::MeshPtr ConvertArtecFrameMeshToRR(artec::sdk::base::IFrameMesh* mesh);

    com::robotraconteur::geometry::shapes::MeshPtr ConvertArtecCompositeMeshToRR(artec::sdk::base::ICompositeMesh* mesh);

    RobotRaconteur::RRArrayPtr<uint8_t> ConvertArtecFrameMeshToObjBytes(artec::sdk::base::IFrameMesh* mesh);

    RobotRaconteur::RRArrayPtr<uint8_t> ConvertArtecCompositeMeshToObjBytes(artec::sdk::base::ICompositeMesh* mesh);

    com::robotraconteur::geometry::Transform ConvertArtecTransformToRR(const artec::sdk::base::Matrix4x4D& transform);

    void ThrowArtecErrorCode(artec::sdk::base::ErrorCode ec, const std::string& user_msg);

    RobotRaconteur::RobotRaconteurExceptionPtr ArtecErrorToExceptionPtr(artec::sdk::base::ErrorCode ec, const std::string& user_msg);

    void ArtecErrorCodeMessage(artec::sdk::base::ErrorCode ec, std::string& msg, std::string& suberr);
    
    std::string ArtecErrorCodeLogMessage(artec::sdk::base::ErrorCode ec);

}