// Auto-generated. Do not edit!

// (in-package rplidar_ros.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class coordinate_msg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.coordinate_x = null;
      this.coordinate_y = null;
      this.angle = null;
    }
    else {
      if (initObj.hasOwnProperty('coordinate_x')) {
        this.coordinate_x = initObj.coordinate_x
      }
      else {
        this.coordinate_x = 0.0;
      }
      if (initObj.hasOwnProperty('coordinate_y')) {
        this.coordinate_y = initObj.coordinate_y
      }
      else {
        this.coordinate_y = 0.0;
      }
      if (initObj.hasOwnProperty('angle')) {
        this.angle = initObj.angle
      }
      else {
        this.angle = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type coordinate_msg
    // Serialize message field [coordinate_x]
    bufferOffset = _serializer.float64(obj.coordinate_x, buffer, bufferOffset);
    // Serialize message field [coordinate_y]
    bufferOffset = _serializer.float64(obj.coordinate_y, buffer, bufferOffset);
    // Serialize message field [angle]
    bufferOffset = _serializer.float64(obj.angle, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type coordinate_msg
    let len;
    let data = new coordinate_msg(null);
    // Deserialize message field [coordinate_x]
    data.coordinate_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [coordinate_y]
    data.coordinate_y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [angle]
    data.angle = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rplidar_ros/coordinate_msg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7e3aeddc9dae72f8917acd183d2e2c53';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 coordinate_x
    float64 coordinate_y
    float64 angle
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new coordinate_msg(null);
    if (msg.coordinate_x !== undefined) {
      resolved.coordinate_x = msg.coordinate_x;
    }
    else {
      resolved.coordinate_x = 0.0
    }

    if (msg.coordinate_y !== undefined) {
      resolved.coordinate_y = msg.coordinate_y;
    }
    else {
      resolved.coordinate_y = 0.0
    }

    if (msg.angle !== undefined) {
      resolved.angle = msg.angle;
    }
    else {
      resolved.angle = 0.0
    }

    return resolved;
    }
};

module.exports = coordinate_msg;
