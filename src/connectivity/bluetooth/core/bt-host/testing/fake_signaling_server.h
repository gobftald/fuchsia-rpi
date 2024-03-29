// Copyright 2020 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef SRC_CONNECTIVITY_BLUETOOTH_CORE_BT_HOST_TESTING_FAKE_SIGNALING_SERVER_H_
#define SRC_CONNECTIVITY_BLUETOOTH_CORE_BT_HOST_TESTING_FAKE_SIGNALING_SERVER_H_

#include "fake_dynamic_channel.h"
#include "fake_l2cap.h"
#include "src/connectivity/bluetooth/core/bt-host/common/byte_buffer.h"
#include "src/connectivity/bluetooth/core/bt-host/common/packet_view.h"
#include "src/connectivity/bluetooth/core/bt-host/hci/hci.h"
#include "src/connectivity/bluetooth/core/bt-host/l2cap/l2cap.h"

namespace bt {
namespace testing {

// This class unpacks data units received from ACL-U and LE-U logical links
// into L2CAP SDUs and then routes them to indvidually-registered handler
// functions. Each FakePeer should have its own FakeL2cap instance and its
// own set of ACL-U and LE-U logical links.

// This class unpacks signaling packets (generally received over a FakeL2cap
// link). Each FakePeer should own its own FakeSignalingServer and should
// instantiate it with its own SendFrameCallback method that
class FakeSignalingServer final {
 public:
  // Entities that instantiate FakeSignalingServer must provide a
  // SendFrameCallback function to handle adding necessary protocol data unit
  // header information to the packet and actually sending the packet using
  // the associated device.
  using SendFrameCallback = fit::function<void(hci::ConnectionHandle conn, const ByteBuffer& sdu)>;

  // Calls |send_frame_callback| with response signaling packets associated
  // with requests received by means of the Handledu method.
  // Has no default value.
  explicit FakeSignalingServer(SendFrameCallback send_frame_callback);

  // Registers this FakeSignalingServer's HandleSdu function with |l2cap_| on
  // kSignalingChannelId such that all packets processed by |l2cap_| with the
  // ChannelId kSignalingChanneld will be processed by this server.
  void RegisterWithL2cap(FakeL2cap* l2cap_);

  // Handles the service data unit |sdu| received over link with handle |conn|
  // by confirming that the received packet is valid and then calling
  // ProcessSignalingPacket.
  void HandleSdu(hci::ConnectionHandle conn, const ByteBuffer& sdu);

  // Parses the InformationRequest signaling packet |info_req| and then
  // calls the appropriate function to construct and send a response packet
  // over handle |conn| and ID |id|.
  void ProcessInformationRequest(hci::ConnectionHandle conn, l2cap::CommandId id,
                                 const ByteBuffer& info_req);

  // Handle incoming ConnectionRequest |connection_req| by validating the
  // request, creating a FakeDynamicChannel object, and registering that
  // channel with the associated FakeL2cap module. Also will use the server's
  // SendFrameCallback to send a ConnectionResponse and ConfiguratioNRequest
  // over handle |conn| and ID |id|
  void ProcessConnectionRequest(hci::ConnectionHandle conn, l2cap::CommandId id,
                                const ByteBuffer& connection_req);

  // Handle incoming ConfigurationRequest |configuration_req|. Note that
  // because the all channels here are basic mode, this is simply part of the
  // connection process and does not actually configure specific parameters
  // of the associated FakeDynamicChannel aside from enabling data transfer
  // when both sides send and receive connection requests.
  // The emulator assumes that the ProcessConnectionRequest function handles
  // sending the initial ConfigurationRequest from FakePeer, so if the emulator
  // received a ConfigurationRequest from bt-host, it will assume the
  // channel is ready to open.
  // FakePeer will still respond with a ConfigurationResponse over handle
  // |conn| using the ID |id| and send it using the server's SendFrameCallback.
  void ProcessConfigurationRequest(hci::ConnectionHandle conn, l2cap::CommandId id,
                                   const ByteBuffer& configuration_req);

  // Handle configuration responses sent by bt-host. The emulator assumes that the
  // ProcessConnectionRequest function handles sending the initial
  // ConfigurationRequest from FakePeer, so it disregarsds this
  // ConnfigurationResponse and instead only processes bt-host's inbound
  // ConfigurationRequest.
  void ProcessConfigurationResponse(hci::ConnectionHandle conn, l2cap::CommandId id,
                                    const ByteBuffer& configuration_res);

  // Upon receiving the disconnection request |disconnection_req|, close and
  // delete the associated channel from the map associated with fake_l2cap_,
  // and then send back a disconnection response with connection handle
  //  |conn| and ID |id| and send it using the server's SendFrameCallback.
  void ProcessDisconnectionRequest(hci::ConnectionHandle conn, l2cap::CommandId id,
                                   const ByteBuffer& disconnection_req);

  // Helper function for sending an individual payload buffer |payload_buffer|
  // by assembling a header with CommandCode |code| and CommandId |id| and then
  // send it over handle |conn| using the send_frame_callback.
  void SendCFrame(hci::ConnectionHandle conn, l2cap::CommandCode code, l2cap::CommandId id, DynamicByteBuffer& payload_buffer);
  
  // Reject a command packet for some |reason|. Assemble a command reject
  // packet using the handle |conn| and the CommandId |id| and send with the
  // FakeSignalingServer's send_frame_callback.
  void SendCommandReject(hci::ConnectionHandle conn, l2cap::CommandId id, l2cap::RejectReason reason);

  // Respond to a received fixed channels InformationRequest packet with a
  // InformationResponse. Assemble the InformationResponse using the handle
  // |conn| and id |id|. Send using the FakeSignalingServer's associated
  // SendFrameCallback.
  void SendInformationResponseFixedChannels(hci::ConnectionHandle conn, l2cap::CommandId id);

  // Respond to a received extended features InformationRequest packet with a
  // InformationResponse. Assemble the InformationResponse using the handle
  // |conn| and id |id|. Send using the FakeSignalingServer's associated
  // SendFrameCallback.
  void SendInformationResponseExtendedFeatures(hci::ConnectionHandle conn, l2cap::CommandId id);

  // Respond to a received ConnectionRequest packet with a ConnectionResponse.
  // Assemble the ConnectionResponse using the handle |conn|, id |id|, local
  // channel |local_id|, remote channel |remote_id|, ConnectionResult |result|,
  // and ConnectionStatus |status|. Send using the FakeSignalingServer's
  // associated SendFrameCallback.
  void SendConnectionResponse(hci::ConnectionHandle conn, l2cap::CommandId id,
                              l2cap::ChannelId local_cid, l2cap::ChannelId remote_id,
                              l2cap::ConnectionResult result, l2cap::ConnectionStatus status);

  // Send a ConfigurationRequest packet following a ConnectionResponse.
  // Assemble the ConfigurationRequest using the handle |conn|, id |id|,
  // and remote ID |remote_cid|. Send using the FakeSignalingServer's
  // associated SendFrameCallback.
  void SendConfigurationRequest(hci::ConnectionHandle conn, l2cap::CommandId id,
                                l2cap::ChannelId remote_cid);

  // Respond to a received ConfigurationRequest packet with a ConfigurationResposne.
  // Assemble the ConfigurationResposne using the handle |conn|, id |id|,
  // local ID |local_cid|, and ConfigurationResult |result|. Send using the
  // FakeSignalingServer's associated SendFrameCallback.
  void SendConfigurationResponse(hci::ConnectionHandle conn, l2cap::CommandId id,
                                 l2cap::ChannelId local_cid, l2cap::ConfigurationResult result);

  // Respond to a received DisconnectionRequest packet with a DisconnectionResponse.
  // Assemble the DisconnectionREsponse using the handle |conn|, id |id|,
  // local ID |local_cid|, and remote ID |remote_cid|. Send using the
  // FakeSignalingServer's associated SendFrameCallback.
  void SendDisconnectionResponse(hci::ConnectionHandle conn, l2cap::CommandId id,
                                 l2cap::ChannelId local_cid, l2cap::ChannelId remote_cid);

  // Return the FakeL2cap instance associated with this FakeSignalingServer.
  FakeL2cap* fake_l2cap() { return fake_l2cap_; }

 private:
  // Function to send signaling packets after the server constructs them.
  SendFrameCallback send_frame_callback_;

  // FakeL2cap instance associated with this server, instantiated after this
  // FakeSignalingServer registers itself with FakeL2cap.
  FakeL2cap* fake_l2cap_;

  DISALLOW_COPY_AND_ASSIGN_ALLOW_MOVE(FakeSignalingServer);
};

}  // namespace testing
}  // namespace bt

#endif  // SRC_CONNECTIVITY_BLUETOOTH_CORE_BT_HOST_TESTING_FAKE_SIGNALING_SERVER_H_
