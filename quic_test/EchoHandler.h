/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

#pragma once

#include <quic/api/QuicSocket.h>

#include <quic/common/BufUtil.h>

// const int RELIABLE_DATA_SIZE = 1 * 1024 * 1024;
// const int UNRELIABLE_DATA_SIZE = 2 * 1024 * 1024;

namespace quic {
namespace samples {
class EchoHandler : public quic::QuicSocket::ConnectionSetupCallback,
                    public quic::QuicSocket::ConnectionCallback,
                    public quic::QuicSocket::ReadCallback,
                    public quic::QuicSocket::WriteCallback,
                    public quic::QuicSocket::DatagramCallback {
 public:
  using StreamData = std::pair<BufQueue, bool>;

  explicit EchoHandler(
      folly::EventBase* evbIn,
      bool useDatagrams = false,
      bool disableRtx = false)
      : evb(evbIn), useDatagrams_(useDatagrams), disableRtx_(disableRtx) {
        
        tcpDataBuffer = std::vector<char>(RELIABLE_DATA_SIZE, 'T');
        udpDataBuffer = std::vector<char>(UNRELIABLE_DATA_SIZE, 'U');
        tcppudpDataBuffer = std::vector<char>(RELIABLE_DATA_SIZE+UNRELIABLE_DATA_SIZE, 'T');
      }

  void setQuicSocket(std::shared_ptr<quic::QuicSocket> socket) {
    sock = socket;
    if (useDatagrams_) {
      auto res = sock->setDatagramCallback(this);
      CHECK(res.hasValue()) << res.error();
    }
  }

  void onNewBidirectionalStream(quic::StreamId id) noexcept override {
    LOG(INFO) << "Got bidirectional stream id=" << id;
    sock->setStreamPriority(id, Priority(1, 0));
    sock->setReadCallback(id, this);
  }

  void onNewBidirectionalStreamGroup(
      quic::StreamGroupId groupId) noexcept override {
    LOG(INFO) << "Got bidirectional stream group id=" << groupId;
    CHECK(streamGroupsData_.find(groupId) == streamGroupsData_.cend());
    streamGroupsData_.emplace(groupId, PerStreamData{});
    if (disableRtx_) {
      QuicStreamGroupRetransmissionPolicy policy;
      policy.disableRetransmission = true;
      sock->setStreamGroupRetransmissionPolicy(groupId, policy);
    }
  }

  void onNewBidirectionalStreamInGroup(
      quic::StreamId id,
      quic::StreamGroupId groupId) noexcept override {
    LOG(INFO) << "Got bidirectional stream id=" << id
              << " in group=" << groupId;
    sock->setReadCallback(id, this);
  }

  void onNewUnidirectionalStream(quic::StreamId id) noexcept override {
    LOG(INFO) << "Got unidirectional stream id=" << id;
    sock->setReadCallback(id, this);
  }

  void onNewUnidirectionalStreamGroup(
      quic::StreamGroupId groupId) noexcept override {
    LOG(INFO) << "Got unidirectional stream group id=" << groupId;
    CHECK(streamGroupsData_.find(groupId) == streamGroupsData_.cend());
    streamGroupsData_.emplace(groupId, PerStreamData{});
  }

  void onNewUnidirectionalStreamInGroup(
      quic::StreamId id,
      quic::StreamGroupId groupId) noexcept override {
    LOG(INFO) << "Got unidirectional stream id=" << id
              << " in group=" << groupId;
    sock->setReadCallback(id, this);
  }

  void onStopSending(
      quic::StreamId id,
      quic::ApplicationErrorCode error) noexcept override {
    LOG(INFO) << "Got StopSending stream id=" << id << " error=" << error;
  }

  void onConnectionEnd() noexcept override {
    LOG(INFO) << "Socket closed";
  }

  void onConnectionSetupError(QuicError error) noexcept override {
    onConnectionError(std::move(error));
  }

  void onConnectionError(QuicError error) noexcept override {
    LOG(ERROR) << "Socket error=" << toString(error.code) << " "
               << error.message;
  }

  void readAvailable(quic::StreamId id) noexcept override {
    LOG(INFO) << "read available for stream id=" << id;

    auto res = sock->read(id, 0);
    if (res.hasError()) {
      LOG(ERROR) << "Got error=" << toString(res.error());
      sock->setReadCallback(id, nullptr);
      return;
    }
    if (input_.find(id) == input_.end()) {
      input_.emplace(id, std::make_pair(BufQueue(), false));
    }
    quic::Buf data = std::move(res.value().first);
    bool eof = res.value().second;
    auto dataLen = (data ? data->computeChainDataLength() : 0);
    auto message = (data) ? data->clone()->moveToFbString().toStdString()
                         : std::string();
    LOG(INFO) << "Got len=" << dataLen << " eof=" << uint32_t(eof)
              << " total=" << input_[id].first.chainLength() + dataLen
              << " data="
              << message;
    input_[id].first.append(std::move(data));
    input_[id].second = eof;
    if (eof) {
      if (message == tcp_p_tcp_scheme) {
        echo(id, input_[id], tcppudpDataBuffer);
      } else {
        echo(id, input_[id], tcpDataBuffer);
        echoDg(udpDataBuffer);
      }
      LOG(INFO) << "uninstalling read callback";
      sock->setReadCallback(id, nullptr);
    }
  }

  void readAvailableWithGroup(
      quic::StreamId id,
      quic::StreamGroupId groupId) noexcept override {
    LOG(INFO) << "read available for stream id=" << id
              << "; groupId=" << groupId;

    auto it = streamGroupsData_.find(groupId);
    CHECK(it != streamGroupsData_.end());

    auto res = sock->read(id, 0);
    if (res.hasError()) {
      LOG(ERROR) << "Got error=" << toString(res.error());
      return;
    }

    auto& streamData = it->second;
    if (streamData.find(id) == streamData.end()) {
      streamData.emplace(id, std::make_pair(BufQueue(), false));
    }

    quic::Buf data = std::move(res.value().first);
    bool eof = res.value().second;
    auto dataLen = (data ? data->computeChainDataLength() : 0);
    LOG(INFO) << "Got len=" << dataLen << " eof=" << uint32_t(eof)
              << " total=" << input_[id].first.chainLength() + dataLen
              << " data="
              << ((data) ? data->clone()->moveToFbString().toStdString()
                         : std::string());

    streamData[id].first.append(std::move(data));
    streamData[id].second = eof;
    if (eof) {
      echo(id, streamData[id], tcpDataBuffer);
    }
  }

  void readError(quic::StreamId id, QuicError error) noexcept override {
    LOG(ERROR) << "Got read error on stream=" << id
               << " error=" << toString(error);
    // A read error only terminates the ingress portion of the stream state.
    // Your application should probably terminate the egress portion via
    // resetStream
  }

  void readErrorWithGroup(
      quic::StreamId id,
      quic::StreamGroupId groupId,
      QuicError error) noexcept override {
    LOG(ERROR) << "Got read error on stream=" << id << "; group=" << groupId
               << " error=" << toString(error);
  }

  void onDatagramsAvailable() noexcept override {
    auto res = sock->readDatagrams();
    if (res.hasError()) {
      LOG(ERROR) << "readDatagrams() error: " << res.error();
      return;
    }
    LOG(INFO) << "received " << res->size() << " datagrams";
    echoDg(udpDataBuffer, std::move(res.value()));
  }

  void onStreamWriteReady(quic::StreamId id, uint64_t maxToSend) noexcept
      override {
    LOG(INFO) << "socket is write ready with maxToSend=" << maxToSend;
    echo(id, input_[id], tcpDataBuffer);
  }

  void onStreamWriteError(quic::StreamId id, QuicError error) noexcept
      override {
    LOG(ERROR) << "write error with stream=" << id
               << " error=" << toString(error);
  }

  folly::EventBase* getEventBase() {
    return evb;
  }

  folly::EventBase* evb;
  std::shared_ptr<quic::QuicSocket> sock;

 private:
  void echo(quic::StreamId id, StreamData& data, std::vector<char>& dataToSend) {
    LOG(INFO) << "In EchoTest" ;
    if (!data.second) {
      // only echo when eof is present
      return;
    }

    auto echoedData = folly::IOBuf::copyBuffer(dataToSend.data(), dataToSend.size());
    // echoedData->prependChain(data.first.move());
    auto res = sock->writeChain(id, std::move(echoedData), true, nullptr);
    if (res.hasError()) {
      LOG(ERROR) << "write error=" << toString(res.error());
    } else {
      // echo is done, clear EOF
      data.second = false;
    }
  }

  void echoDg(std::vector<char>& dataToSend, std::vector<quic::ReadDatagram> datagrams={}) {
    size_t processed = 0;
    bool errorOccured = false;
    while (processed < (int)dataToSend.size()) {
      auto echoedData = folly::IOBuf::copyBuffer(dataToSend.data() + processed, std::min(1200, (int)dataToSend.size()-(int)processed));

      auto res = sock->writeDatagram(std::move(echoedData));
      if (res.hasError() && !errorOccured) {
        LOG(ERROR) << "Error point: " << processed << " / " << (int)dataToSend.size();
        LOG(ERROR) << "writeDatagram error=" << toString(res.error());
        return;
      }
      processed += 1200;
    }
  }

  bool useDatagrams_;
  std::vector<char> tcpDataBuffer;
  std::vector<char> udpDataBuffer;
  std::vector<char> tcppudpDataBuffer;
  using PerStreamData = std::map<quic::StreamId, StreamData>;
  PerStreamData input_;
  std::map<quic::StreamGroupId, PerStreamData> streamGroupsData_;
  bool disableRtx_{false};
};
} // namespace samples
} // namespace quic
