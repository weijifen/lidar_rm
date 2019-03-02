// Auto-generated. Do not edit!

// (in-package serial_common.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class wind_mill_msg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.horizonal = null;
      this.vertical = null;
    }
    else {
      if (initObj.hasOwnProperty('horizonal')) {
        this.horizonal = initObj.horizonal
      }
      else {
        this.horizonal = 0.0;
      }
      if (initObj.hasOwnProperty('vertical')) {
        this.vertical = initObj.vertical
      }
      else {
        this.vertical = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type wind_mill_msg
    // Serialize message field [horizonal]
    bufferOffset = _serializer.float32(obj.horizonal, buffer, bufferOffset);
    // Serialize message field [vertical]
    bufferOffset = _serializer.float32(obj.vertical, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type wind_mill_msg
    let len;
    let data = new wind_mill_msg(null);
    // Deserialize message field [horizonal]
    data.horizonal = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vertical]
    data.vertical = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'serial_common/wind_mill_msg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cac8e6416acc3c4967ad84a4b158da62';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 horizonal
    float32 vertical
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new wind_mill_msg(null);
    if (msg.horizonal !== undefined) {
      resolved.horizonal = msg.horizonal;
    }
    else {
      resolved.horizonal = 0.0
    }

    if (msg.vertical !== undefined) {
      resolved.vertical = msg.vertical;
    }
    else {
      resolved.vertical = 0.0
    }

    return resolved;
    }
};

module.exports = wind_mill_msg;
