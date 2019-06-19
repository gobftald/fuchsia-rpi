// WARNING: This file is machine generated by fidlgen.

#include <fidl_llcpp_basictypes.h>
#include <memory>

namespace llcpp {

namespace fidl {
namespace test {
namespace llcpp {
namespace basictypes {

::llcpp::fidl::test::llcpp::basictypes::SimpleUnion::SimpleUnion() {
  tag_ = Tag::Invalid;
}

::llcpp::fidl::test::llcpp::basictypes::SimpleUnion::~SimpleUnion() {
  Destroy();
}

void ::llcpp::fidl::test::llcpp::basictypes::SimpleUnion::Destroy() {
  switch (which()) {
  default:
    break;
  }
  tag_ = Tag::Invalid;
}

void ::llcpp::fidl::test::llcpp::basictypes::SimpleUnion::MoveImpl_(SimpleUnion&& other) {
  switch (other.which()) {
  case Tag::kFieldA:
    mutable_field_a() = std::move(other.mutable_field_a());
    break;
  case Tag::kFieldB:
    mutable_field_b() = std::move(other.mutable_field_b());
    break;
  default:
    break;
  }
  other.Destroy();
}

void ::llcpp::fidl::test::llcpp::basictypes::SimpleUnion::SizeAndOffsetAssertionHelper() {
  static_assert(offsetof(::llcpp::fidl::test::llcpp::basictypes::SimpleUnion, field_a_) == 4);
  static_assert(offsetof(::llcpp::fidl::test::llcpp::basictypes::SimpleUnion, field_b_) == 4);
  static_assert(sizeof(::llcpp::fidl::test::llcpp::basictypes::SimpleUnion) == ::llcpp::fidl::test::llcpp::basictypes::SimpleUnion::PrimarySize);
}


int32_t& ::llcpp::fidl::test::llcpp::basictypes::SimpleUnion::mutable_field_a() {
  if (which() != Tag::kFieldA) {
    Destroy();
    new (&field_a_) int32_t;
  }
  tag_ = Tag::kFieldA;
  return field_a_;
}

int32_t& ::llcpp::fidl::test::llcpp::basictypes::SimpleUnion::mutable_field_b() {
  if (which() != Tag::kFieldB) {
    Destroy();
    new (&field_b_) int32_t;
  }
  tag_ = Tag::kFieldB;
  return field_b_;
}


namespace {

[[maybe_unused]]
constexpr uint32_t kTestInterface_ConsumeSimpleStruct_Ordinal = 728053387u;
extern "C" const fidl_type_t fidl_test_llcpp_basictypes_TestInterfaceConsumeSimpleStructRequestTable;
[[maybe_unused]]
constexpr uint32_t kTestInterface_ConsumeSimpleUnion_Ordinal = 776403313u;

}  // namespace

zx_status_t TestInterface::SyncClient::ConsumeSimpleStruct(SimpleStruct arg, int32_t* out_status, int32_t* out_field) {
  return TestInterface::Call::ConsumeSimpleStruct(zx::unowned_channel(this->channel_), std::move(arg), out_status, out_field);
}

zx_status_t TestInterface::Call::ConsumeSimpleStruct(zx::unowned_channel _client_end, SimpleStruct arg, int32_t* out_status, int32_t* out_field) {
  constexpr uint32_t _kWriteAllocSize = ::fidl::internal::ClampedMessageSize<ConsumeSimpleStructRequest>();
  FIDL_ALIGNDECL uint8_t _write_bytes[_kWriteAllocSize] = {};
  auto& _request = *reinterpret_cast<ConsumeSimpleStructRequest*>(_write_bytes);
  _request._hdr.ordinal = kTestInterface_ConsumeSimpleStruct_Ordinal;
  _request.arg = std::move(arg);
  ::fidl::BytePart _request_bytes(_write_bytes, _kWriteAllocSize, sizeof(ConsumeSimpleStructRequest));
  ::fidl::DecodedMessage<ConsumeSimpleStructRequest> _decoded_request(std::move(_request_bytes));
  auto _encode_request_result = ::fidl::Encode(std::move(_decoded_request));
  if (_encode_request_result.status != ZX_OK) {
    return _encode_request_result.status;
  }
  constexpr uint32_t _kReadAllocSize = ::fidl::internal::ClampedMessageSize<ConsumeSimpleStructResponse>();
  FIDL_ALIGNDECL uint8_t _read_bytes[_kReadAllocSize];
  ::fidl::BytePart _response_bytes(_read_bytes, _kReadAllocSize);
  auto _call_result = ::fidl::Call<ConsumeSimpleStructRequest, ConsumeSimpleStructResponse>(
    std::move(_client_end), std::move(_encode_request_result.message), std::move(_response_bytes));
  if (_call_result.status != ZX_OK) {
    return _call_result.status;
  }
  auto _decode_result = ::fidl::Decode(std::move(_call_result.message));
  if (_decode_result.status != ZX_OK) {
    return _decode_result.status;
  }
  auto& _response = *_decode_result.message.message();
  *out_status = std::move(_response.status);
  *out_field = std::move(_response.field);
  return ZX_OK;
}

::fidl::DecodeResult<TestInterface::ConsumeSimpleStructResponse> TestInterface::SyncClient::ConsumeSimpleStruct(::fidl::BytePart _request_buffer, SimpleStruct arg, ::fidl::BytePart _response_buffer, int32_t* out_status, int32_t* out_field) {
  return TestInterface::Call::ConsumeSimpleStruct(zx::unowned_channel(this->channel_), std::move(_request_buffer), std::move(arg), std::move(_response_buffer), out_status, out_field);
}

::fidl::DecodeResult<TestInterface::ConsumeSimpleStructResponse> TestInterface::Call::ConsumeSimpleStruct(zx::unowned_channel _client_end, ::fidl::BytePart _request_buffer, SimpleStruct arg, ::fidl::BytePart _response_buffer, int32_t* out_status, int32_t* out_field) {
  if (_request_buffer.capacity() < ConsumeSimpleStructRequest::PrimarySize) {
    return ::fidl::DecodeResult<ConsumeSimpleStructResponse>(ZX_ERR_BUFFER_TOO_SMALL, ::fidl::internal::kErrorRequestBufferTooSmall);
  }
  auto& _request = *reinterpret_cast<ConsumeSimpleStructRequest*>(_request_buffer.data());
  _request._hdr.ordinal = kTestInterface_ConsumeSimpleStruct_Ordinal;
  _request.arg = std::move(arg);
  _request_buffer.set_actual(sizeof(ConsumeSimpleStructRequest));
  ::fidl::DecodedMessage<ConsumeSimpleStructRequest> _decoded_request(std::move(_request_buffer));
  auto _encode_request_result = ::fidl::Encode(std::move(_decoded_request));
  if (_encode_request_result.status != ZX_OK) {
    return ::fidl::DecodeResult<ConsumeSimpleStructResponse>(_encode_request_result.status, _encode_request_result.error);
  }
  auto _call_result = ::fidl::Call<ConsumeSimpleStructRequest, ConsumeSimpleStructResponse>(
    std::move(_client_end), std::move(_encode_request_result.message), std::move(_response_buffer));
  if (_call_result.status != ZX_OK) {
    return ::fidl::DecodeResult<ConsumeSimpleStructResponse>(_call_result.status, _call_result.error);
  }
  auto _decode_result = ::fidl::Decode(std::move(_call_result.message));
  if (_decode_result.status != ZX_OK) {
    return _decode_result;
  }
  auto& _response = *_decode_result.message.message();
  *out_status = std::move(_response.status);
  *out_field = std::move(_response.field);
  return _decode_result;
}

::fidl::DecodeResult<TestInterface::ConsumeSimpleStructResponse> TestInterface::SyncClient::ConsumeSimpleStruct(::fidl::DecodedMessage<ConsumeSimpleStructRequest> params, ::fidl::BytePart response_buffer) {
  return TestInterface::Call::ConsumeSimpleStruct(zx::unowned_channel(this->channel_), std::move(params), std::move(response_buffer));
}

::fidl::DecodeResult<TestInterface::ConsumeSimpleStructResponse> TestInterface::Call::ConsumeSimpleStruct(zx::unowned_channel _client_end, ::fidl::DecodedMessage<ConsumeSimpleStructRequest> params, ::fidl::BytePart response_buffer) {
  params.message()->_hdr = {};
  params.message()->_hdr.ordinal = kTestInterface_ConsumeSimpleStruct_Ordinal;
  auto _encode_request_result = ::fidl::Encode(std::move(params));
  if (_encode_request_result.status != ZX_OK) {
    return ::fidl::DecodeResult<TestInterface::ConsumeSimpleStructResponse>(
      _encode_request_result.status,
      _encode_request_result.error,
      ::fidl::DecodedMessage<TestInterface::ConsumeSimpleStructResponse>());
  }
  auto _call_result = ::fidl::Call<ConsumeSimpleStructRequest, ConsumeSimpleStructResponse>(
    std::move(_client_end), std::move(_encode_request_result.message), std::move(response_buffer));
  if (_call_result.status != ZX_OK) {
    return ::fidl::DecodeResult<TestInterface::ConsumeSimpleStructResponse>(
      _call_result.status,
      _call_result.error,
      ::fidl::DecodedMessage<TestInterface::ConsumeSimpleStructResponse>());
  }
  return ::fidl::Decode(std::move(_call_result.message));
}


zx_status_t TestInterface::SyncClient::ConsumeSimpleUnion(SimpleUnion arg, uint32_t* out_index, int32_t* out_field) {
  return TestInterface::Call::ConsumeSimpleUnion(zx::unowned_channel(this->channel_), std::move(arg), out_index, out_field);
}

zx_status_t TestInterface::Call::ConsumeSimpleUnion(zx::unowned_channel _client_end, SimpleUnion arg, uint32_t* out_index, int32_t* out_field) {
  constexpr uint32_t _kWriteAllocSize = ::fidl::internal::ClampedMessageSize<ConsumeSimpleUnionRequest>();
  FIDL_ALIGNDECL uint8_t _write_bytes[_kWriteAllocSize] = {};
  auto& _request = *reinterpret_cast<ConsumeSimpleUnionRequest*>(_write_bytes);
  _request._hdr.ordinal = kTestInterface_ConsumeSimpleUnion_Ordinal;
  _request.arg = std::move(arg);
  ::fidl::BytePart _request_bytes(_write_bytes, _kWriteAllocSize, sizeof(ConsumeSimpleUnionRequest));
  ::fidl::DecodedMessage<ConsumeSimpleUnionRequest> _decoded_request(std::move(_request_bytes));
  auto _encode_request_result = ::fidl::Encode(std::move(_decoded_request));
  if (_encode_request_result.status != ZX_OK) {
    return _encode_request_result.status;
  }
  constexpr uint32_t _kReadAllocSize = ::fidl::internal::ClampedMessageSize<ConsumeSimpleUnionResponse>();
  FIDL_ALIGNDECL uint8_t _read_bytes[_kReadAllocSize];
  ::fidl::BytePart _response_bytes(_read_bytes, _kReadAllocSize);
  auto _call_result = ::fidl::Call<ConsumeSimpleUnionRequest, ConsumeSimpleUnionResponse>(
    std::move(_client_end), std::move(_encode_request_result.message), std::move(_response_bytes));
  if (_call_result.status != ZX_OK) {
    return _call_result.status;
  }
  auto _decode_result = ::fidl::Decode(std::move(_call_result.message));
  if (_decode_result.status != ZX_OK) {
    return _decode_result.status;
  }
  auto& _response = *_decode_result.message.message();
  *out_index = std::move(_response.index);
  *out_field = std::move(_response.field);
  return ZX_OK;
}

::fidl::DecodeResult<TestInterface::ConsumeSimpleUnionResponse> TestInterface::SyncClient::ConsumeSimpleUnion(::fidl::BytePart _request_buffer, SimpleUnion arg, ::fidl::BytePart _response_buffer, uint32_t* out_index, int32_t* out_field) {
  return TestInterface::Call::ConsumeSimpleUnion(zx::unowned_channel(this->channel_), std::move(_request_buffer), std::move(arg), std::move(_response_buffer), out_index, out_field);
}

::fidl::DecodeResult<TestInterface::ConsumeSimpleUnionResponse> TestInterface::Call::ConsumeSimpleUnion(zx::unowned_channel _client_end, ::fidl::BytePart _request_buffer, SimpleUnion arg, ::fidl::BytePart _response_buffer, uint32_t* out_index, int32_t* out_field) {
  if (_request_buffer.capacity() < ConsumeSimpleUnionRequest::PrimarySize) {
    return ::fidl::DecodeResult<ConsumeSimpleUnionResponse>(ZX_ERR_BUFFER_TOO_SMALL, ::fidl::internal::kErrorRequestBufferTooSmall);
  }
  auto& _request = *reinterpret_cast<ConsumeSimpleUnionRequest*>(_request_buffer.data());
  _request._hdr.ordinal = kTestInterface_ConsumeSimpleUnion_Ordinal;
  _request.arg = std::move(arg);
  _request_buffer.set_actual(sizeof(ConsumeSimpleUnionRequest));
  ::fidl::DecodedMessage<ConsumeSimpleUnionRequest> _decoded_request(std::move(_request_buffer));
  auto _encode_request_result = ::fidl::Encode(std::move(_decoded_request));
  if (_encode_request_result.status != ZX_OK) {
    return ::fidl::DecodeResult<ConsumeSimpleUnionResponse>(_encode_request_result.status, _encode_request_result.error);
  }
  auto _call_result = ::fidl::Call<ConsumeSimpleUnionRequest, ConsumeSimpleUnionResponse>(
    std::move(_client_end), std::move(_encode_request_result.message), std::move(_response_buffer));
  if (_call_result.status != ZX_OK) {
    return ::fidl::DecodeResult<ConsumeSimpleUnionResponse>(_call_result.status, _call_result.error);
  }
  auto _decode_result = ::fidl::Decode(std::move(_call_result.message));
  if (_decode_result.status != ZX_OK) {
    return _decode_result;
  }
  auto& _response = *_decode_result.message.message();
  *out_index = std::move(_response.index);
  *out_field = std::move(_response.field);
  return _decode_result;
}

::fidl::DecodeResult<TestInterface::ConsumeSimpleUnionResponse> TestInterface::SyncClient::ConsumeSimpleUnion(::fidl::DecodedMessage<ConsumeSimpleUnionRequest> params, ::fidl::BytePart response_buffer) {
  return TestInterface::Call::ConsumeSimpleUnion(zx::unowned_channel(this->channel_), std::move(params), std::move(response_buffer));
}

::fidl::DecodeResult<TestInterface::ConsumeSimpleUnionResponse> TestInterface::Call::ConsumeSimpleUnion(zx::unowned_channel _client_end, ::fidl::DecodedMessage<ConsumeSimpleUnionRequest> params, ::fidl::BytePart response_buffer) {
  params.message()->_hdr = {};
  params.message()->_hdr.ordinal = kTestInterface_ConsumeSimpleUnion_Ordinal;
  auto _encode_request_result = ::fidl::Encode(std::move(params));
  if (_encode_request_result.status != ZX_OK) {
    return ::fidl::DecodeResult<TestInterface::ConsumeSimpleUnionResponse>(
      _encode_request_result.status,
      _encode_request_result.error,
      ::fidl::DecodedMessage<TestInterface::ConsumeSimpleUnionResponse>());
  }
  auto _call_result = ::fidl::Call<ConsumeSimpleUnionRequest, ConsumeSimpleUnionResponse>(
    std::move(_client_end), std::move(_encode_request_result.message), std::move(response_buffer));
  if (_call_result.status != ZX_OK) {
    return ::fidl::DecodeResult<TestInterface::ConsumeSimpleUnionResponse>(
      _call_result.status,
      _call_result.error,
      ::fidl::DecodedMessage<TestInterface::ConsumeSimpleUnionResponse>());
  }
  return ::fidl::Decode(std::move(_call_result.message));
}


bool TestInterface::TryDispatch(Interface* impl, fidl_msg_t* msg, ::fidl::Transaction* txn) {
  if (msg->num_bytes < sizeof(fidl_message_header_t)) {
    zx_handle_close_many(msg->handles, msg->num_handles);
    txn->Close(ZX_ERR_INVALID_ARGS);
    return true;
  }
  fidl_message_header_t* hdr = reinterpret_cast<fidl_message_header_t*>(msg->bytes);
  switch (hdr->ordinal) {
    case kTestInterface_ConsumeSimpleStruct_Ordinal: {
      auto result = ::fidl::DecodeAs<ConsumeSimpleStructRequest>(msg);
      if (result.status != ZX_OK) {
        txn->Close(ZX_ERR_INVALID_ARGS);
        return true;
      }
      auto message = result.message.message();
      impl->ConsumeSimpleStruct(std::move(message->arg),
        Interface::ConsumeSimpleStructCompleter::Sync(txn));
      return true;
    }
    case kTestInterface_ConsumeSimpleUnion_Ordinal: {
      auto result = ::fidl::DecodeAs<ConsumeSimpleUnionRequest>(msg);
      if (result.status != ZX_OK) {
        txn->Close(ZX_ERR_INVALID_ARGS);
        return true;
      }
      auto message = result.message.message();
      impl->ConsumeSimpleUnion(std::move(message->arg),
        Interface::ConsumeSimpleUnionCompleter::Sync(txn));
      return true;
    }
    default: {
      return false;
    }
  }
}

bool TestInterface::Dispatch(Interface* impl, fidl_msg_t* msg, ::fidl::Transaction* txn) {
  bool found = TryDispatch(impl, msg, txn);
  if (!found) {
    zx_handle_close_many(msg->handles, msg->num_handles);
    txn->Close(ZX_ERR_NOT_SUPPORTED);
  }
  return found;
}


void TestInterface::Interface::ConsumeSimpleStructCompleterBase::Reply(int32_t status, int32_t field) {
  constexpr uint32_t _kWriteAllocSize = ::fidl::internal::ClampedMessageSize<ConsumeSimpleStructResponse>();
  FIDL_ALIGNDECL uint8_t _write_bytes[_kWriteAllocSize] = {};
  auto& _response = *reinterpret_cast<ConsumeSimpleStructResponse*>(_write_bytes);
  _response._hdr.ordinal = kTestInterface_ConsumeSimpleStruct_Ordinal;
  _response.status = std::move(status);
  _response.field = std::move(field);
  ::fidl::BytePart _response_bytes(_write_bytes, _kWriteAllocSize, sizeof(ConsumeSimpleStructResponse));
  CompleterBase::SendReply(::fidl::DecodedMessage<ConsumeSimpleStructResponse>(std::move(_response_bytes)));
}

void TestInterface::Interface::ConsumeSimpleStructCompleterBase::Reply(::fidl::BytePart _buffer, int32_t status, int32_t field) {
  if (_buffer.capacity() < ConsumeSimpleStructResponse::PrimarySize) {
    CompleterBase::Close(ZX_ERR_INTERNAL);
    return;
  }
  auto& _response = *reinterpret_cast<ConsumeSimpleStructResponse*>(_buffer.data());
  _response._hdr.ordinal = kTestInterface_ConsumeSimpleStruct_Ordinal;
  _response.status = std::move(status);
  _response.field = std::move(field);
  _buffer.set_actual(sizeof(ConsumeSimpleStructResponse));
  CompleterBase::SendReply(::fidl::DecodedMessage<ConsumeSimpleStructResponse>(std::move(_buffer)));
}

void TestInterface::Interface::ConsumeSimpleStructCompleterBase::Reply(::fidl::DecodedMessage<ConsumeSimpleStructResponse> params) {
  params.message()->_hdr = {};
  params.message()->_hdr.ordinal = kTestInterface_ConsumeSimpleStruct_Ordinal;
  CompleterBase::SendReply(std::move(params));
}


void TestInterface::Interface::ConsumeSimpleUnionCompleterBase::Reply(uint32_t index, int32_t field) {
  constexpr uint32_t _kWriteAllocSize = ::fidl::internal::ClampedMessageSize<ConsumeSimpleUnionResponse>();
  FIDL_ALIGNDECL uint8_t _write_bytes[_kWriteAllocSize] = {};
  auto& _response = *reinterpret_cast<ConsumeSimpleUnionResponse*>(_write_bytes);
  _response._hdr.ordinal = kTestInterface_ConsumeSimpleUnion_Ordinal;
  _response.index = std::move(index);
  _response.field = std::move(field);
  ::fidl::BytePart _response_bytes(_write_bytes, _kWriteAllocSize, sizeof(ConsumeSimpleUnionResponse));
  CompleterBase::SendReply(::fidl::DecodedMessage<ConsumeSimpleUnionResponse>(std::move(_response_bytes)));
}

void TestInterface::Interface::ConsumeSimpleUnionCompleterBase::Reply(::fidl::BytePart _buffer, uint32_t index, int32_t field) {
  if (_buffer.capacity() < ConsumeSimpleUnionResponse::PrimarySize) {
    CompleterBase::Close(ZX_ERR_INTERNAL);
    return;
  }
  auto& _response = *reinterpret_cast<ConsumeSimpleUnionResponse*>(_buffer.data());
  _response._hdr.ordinal = kTestInterface_ConsumeSimpleUnion_Ordinal;
  _response.index = std::move(index);
  _response.field = std::move(field);
  _buffer.set_actual(sizeof(ConsumeSimpleUnionResponse));
  CompleterBase::SendReply(::fidl::DecodedMessage<ConsumeSimpleUnionResponse>(std::move(_buffer)));
}

void TestInterface::Interface::ConsumeSimpleUnionCompleterBase::Reply(::fidl::DecodedMessage<ConsumeSimpleUnionResponse> params) {
  params.message()->_hdr = {};
  params.message()->_hdr.ordinal = kTestInterface_ConsumeSimpleUnion_Ordinal;
  CompleterBase::SendReply(std::move(params));
}


}  // namespace basictypes
}  // namespace llcpp
}  // namespace test
}  // namespace fidl
}  // namespace llcpp
