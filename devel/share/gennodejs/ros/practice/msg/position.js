// Auto-generated. Do not edit!

// (in-package practice.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class position {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.uav0_px = null;
      this.uav0_py = null;
      this.uav0_pz = null;
      this.uav1_px = null;
      this.uav1_py = null;
      this.uav1_pz = null;
      this.uav2_px = null;
      this.uav2_py = null;
      this.uav2_pz = null;
      this.g0_px = null;
      this.g0_py = null;
      this.g0_pz = null;
      this.g1_px = null;
      this.g1_py = null;
      this.g1_pz = null;
      this.g2_px = null;
      this.g2_py = null;
      this.g2_pz = null;
      this.time = null;
      this.uav0_pid_vx = null;
      this.uav0_pid_vy = null;
      this.uav0_pid_vz = null;
      this.uav0_aS = null;
      this.uav0_a_vx = null;
      this.uav0_a_vy = null;
      this.uav0_a_vz = null;
      this.uav0_cS = null;
      this.uav0_c_vx = null;
      this.uav0_c_vy = null;
      this.uav0_c_vz = null;
      this.uav1_pid_vx = null;
      this.uav1_pid_vy = null;
      this.uav1_pid_vz = null;
      this.uav1_aS = null;
      this.uav1_a_vx = null;
      this.uav1_a_vy = null;
      this.uav1_a_vz = null;
      this.uav1_cS = null;
      this.uav1_c_vx = null;
      this.uav1_c_vy = null;
      this.uav1_c_vz = null;
      this.uav2_pid_vx = null;
      this.uav2_pid_vy = null;
      this.uav2_pid_vz = null;
      this.uav2_aS = null;
      this.uav2_a_vx = null;
      this.uav2_a_vy = null;
      this.uav2_a_vz = null;
      this.uav2_cS = null;
      this.uav2_c_vx = null;
      this.uav2_c_vy = null;
      this.uav2_c_vz = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('uav0_px')) {
        this.uav0_px = initObj.uav0_px
      }
      else {
        this.uav0_px = 0.0;
      }
      if (initObj.hasOwnProperty('uav0_py')) {
        this.uav0_py = initObj.uav0_py
      }
      else {
        this.uav0_py = 0.0;
      }
      if (initObj.hasOwnProperty('uav0_pz')) {
        this.uav0_pz = initObj.uav0_pz
      }
      else {
        this.uav0_pz = 0.0;
      }
      if (initObj.hasOwnProperty('uav1_px')) {
        this.uav1_px = initObj.uav1_px
      }
      else {
        this.uav1_px = 0.0;
      }
      if (initObj.hasOwnProperty('uav1_py')) {
        this.uav1_py = initObj.uav1_py
      }
      else {
        this.uav1_py = 0.0;
      }
      if (initObj.hasOwnProperty('uav1_pz')) {
        this.uav1_pz = initObj.uav1_pz
      }
      else {
        this.uav1_pz = 0.0;
      }
      if (initObj.hasOwnProperty('uav2_px')) {
        this.uav2_px = initObj.uav2_px
      }
      else {
        this.uav2_px = 0.0;
      }
      if (initObj.hasOwnProperty('uav2_py')) {
        this.uav2_py = initObj.uav2_py
      }
      else {
        this.uav2_py = 0.0;
      }
      if (initObj.hasOwnProperty('uav2_pz')) {
        this.uav2_pz = initObj.uav2_pz
      }
      else {
        this.uav2_pz = 0.0;
      }
      if (initObj.hasOwnProperty('g0_px')) {
        this.g0_px = initObj.g0_px
      }
      else {
        this.g0_px = 0.0;
      }
      if (initObj.hasOwnProperty('g0_py')) {
        this.g0_py = initObj.g0_py
      }
      else {
        this.g0_py = 0.0;
      }
      if (initObj.hasOwnProperty('g0_pz')) {
        this.g0_pz = initObj.g0_pz
      }
      else {
        this.g0_pz = 0.0;
      }
      if (initObj.hasOwnProperty('g1_px')) {
        this.g1_px = initObj.g1_px
      }
      else {
        this.g1_px = 0.0;
      }
      if (initObj.hasOwnProperty('g1_py')) {
        this.g1_py = initObj.g1_py
      }
      else {
        this.g1_py = 0.0;
      }
      if (initObj.hasOwnProperty('g1_pz')) {
        this.g1_pz = initObj.g1_pz
      }
      else {
        this.g1_pz = 0.0;
      }
      if (initObj.hasOwnProperty('g2_px')) {
        this.g2_px = initObj.g2_px
      }
      else {
        this.g2_px = 0.0;
      }
      if (initObj.hasOwnProperty('g2_py')) {
        this.g2_py = initObj.g2_py
      }
      else {
        this.g2_py = 0.0;
      }
      if (initObj.hasOwnProperty('g2_pz')) {
        this.g2_pz = initObj.g2_pz
      }
      else {
        this.g2_pz = 0.0;
      }
      if (initObj.hasOwnProperty('time')) {
        this.time = initObj.time
      }
      else {
        this.time = 0.0;
      }
      if (initObj.hasOwnProperty('uav0_pid_vx')) {
        this.uav0_pid_vx = initObj.uav0_pid_vx
      }
      else {
        this.uav0_pid_vx = 0.0;
      }
      if (initObj.hasOwnProperty('uav0_pid_vy')) {
        this.uav0_pid_vy = initObj.uav0_pid_vy
      }
      else {
        this.uav0_pid_vy = 0.0;
      }
      if (initObj.hasOwnProperty('uav0_pid_vz')) {
        this.uav0_pid_vz = initObj.uav0_pid_vz
      }
      else {
        this.uav0_pid_vz = 0.0;
      }
      if (initObj.hasOwnProperty('uav0_aS')) {
        this.uav0_aS = initObj.uav0_aS
      }
      else {
        this.uav0_aS = 0.0;
      }
      if (initObj.hasOwnProperty('uav0_a_vx')) {
        this.uav0_a_vx = initObj.uav0_a_vx
      }
      else {
        this.uav0_a_vx = 0.0;
      }
      if (initObj.hasOwnProperty('uav0_a_vy')) {
        this.uav0_a_vy = initObj.uav0_a_vy
      }
      else {
        this.uav0_a_vy = 0.0;
      }
      if (initObj.hasOwnProperty('uav0_a_vz')) {
        this.uav0_a_vz = initObj.uav0_a_vz
      }
      else {
        this.uav0_a_vz = 0.0;
      }
      if (initObj.hasOwnProperty('uav0_cS')) {
        this.uav0_cS = initObj.uav0_cS
      }
      else {
        this.uav0_cS = 0.0;
      }
      if (initObj.hasOwnProperty('uav0_c_vx')) {
        this.uav0_c_vx = initObj.uav0_c_vx
      }
      else {
        this.uav0_c_vx = 0.0;
      }
      if (initObj.hasOwnProperty('uav0_c_vy')) {
        this.uav0_c_vy = initObj.uav0_c_vy
      }
      else {
        this.uav0_c_vy = 0.0;
      }
      if (initObj.hasOwnProperty('uav0_c_vz')) {
        this.uav0_c_vz = initObj.uav0_c_vz
      }
      else {
        this.uav0_c_vz = 0.0;
      }
      if (initObj.hasOwnProperty('uav1_pid_vx')) {
        this.uav1_pid_vx = initObj.uav1_pid_vx
      }
      else {
        this.uav1_pid_vx = 0.0;
      }
      if (initObj.hasOwnProperty('uav1_pid_vy')) {
        this.uav1_pid_vy = initObj.uav1_pid_vy
      }
      else {
        this.uav1_pid_vy = 0.0;
      }
      if (initObj.hasOwnProperty('uav1_pid_vz')) {
        this.uav1_pid_vz = initObj.uav1_pid_vz
      }
      else {
        this.uav1_pid_vz = 0.0;
      }
      if (initObj.hasOwnProperty('uav1_aS')) {
        this.uav1_aS = initObj.uav1_aS
      }
      else {
        this.uav1_aS = 0.0;
      }
      if (initObj.hasOwnProperty('uav1_a_vx')) {
        this.uav1_a_vx = initObj.uav1_a_vx
      }
      else {
        this.uav1_a_vx = 0.0;
      }
      if (initObj.hasOwnProperty('uav1_a_vy')) {
        this.uav1_a_vy = initObj.uav1_a_vy
      }
      else {
        this.uav1_a_vy = 0.0;
      }
      if (initObj.hasOwnProperty('uav1_a_vz')) {
        this.uav1_a_vz = initObj.uav1_a_vz
      }
      else {
        this.uav1_a_vz = 0.0;
      }
      if (initObj.hasOwnProperty('uav1_cS')) {
        this.uav1_cS = initObj.uav1_cS
      }
      else {
        this.uav1_cS = 0.0;
      }
      if (initObj.hasOwnProperty('uav1_c_vx')) {
        this.uav1_c_vx = initObj.uav1_c_vx
      }
      else {
        this.uav1_c_vx = 0.0;
      }
      if (initObj.hasOwnProperty('uav1_c_vy')) {
        this.uav1_c_vy = initObj.uav1_c_vy
      }
      else {
        this.uav1_c_vy = 0.0;
      }
      if (initObj.hasOwnProperty('uav1_c_vz')) {
        this.uav1_c_vz = initObj.uav1_c_vz
      }
      else {
        this.uav1_c_vz = 0.0;
      }
      if (initObj.hasOwnProperty('uav2_pid_vx')) {
        this.uav2_pid_vx = initObj.uav2_pid_vx
      }
      else {
        this.uav2_pid_vx = 0.0;
      }
      if (initObj.hasOwnProperty('uav2_pid_vy')) {
        this.uav2_pid_vy = initObj.uav2_pid_vy
      }
      else {
        this.uav2_pid_vy = 0.0;
      }
      if (initObj.hasOwnProperty('uav2_pid_vz')) {
        this.uav2_pid_vz = initObj.uav2_pid_vz
      }
      else {
        this.uav2_pid_vz = 0.0;
      }
      if (initObj.hasOwnProperty('uav2_aS')) {
        this.uav2_aS = initObj.uav2_aS
      }
      else {
        this.uav2_aS = 0.0;
      }
      if (initObj.hasOwnProperty('uav2_a_vx')) {
        this.uav2_a_vx = initObj.uav2_a_vx
      }
      else {
        this.uav2_a_vx = 0.0;
      }
      if (initObj.hasOwnProperty('uav2_a_vy')) {
        this.uav2_a_vy = initObj.uav2_a_vy
      }
      else {
        this.uav2_a_vy = 0.0;
      }
      if (initObj.hasOwnProperty('uav2_a_vz')) {
        this.uav2_a_vz = initObj.uav2_a_vz
      }
      else {
        this.uav2_a_vz = 0.0;
      }
      if (initObj.hasOwnProperty('uav2_cS')) {
        this.uav2_cS = initObj.uav2_cS
      }
      else {
        this.uav2_cS = 0.0;
      }
      if (initObj.hasOwnProperty('uav2_c_vx')) {
        this.uav2_c_vx = initObj.uav2_c_vx
      }
      else {
        this.uav2_c_vx = 0.0;
      }
      if (initObj.hasOwnProperty('uav2_c_vy')) {
        this.uav2_c_vy = initObj.uav2_c_vy
      }
      else {
        this.uav2_c_vy = 0.0;
      }
      if (initObj.hasOwnProperty('uav2_c_vz')) {
        this.uav2_c_vz = initObj.uav2_c_vz
      }
      else {
        this.uav2_c_vz = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type position
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [uav0_px]
    bufferOffset = _serializer.float64(obj.uav0_px, buffer, bufferOffset);
    // Serialize message field [uav0_py]
    bufferOffset = _serializer.float64(obj.uav0_py, buffer, bufferOffset);
    // Serialize message field [uav0_pz]
    bufferOffset = _serializer.float64(obj.uav0_pz, buffer, bufferOffset);
    // Serialize message field [uav1_px]
    bufferOffset = _serializer.float64(obj.uav1_px, buffer, bufferOffset);
    // Serialize message field [uav1_py]
    bufferOffset = _serializer.float64(obj.uav1_py, buffer, bufferOffset);
    // Serialize message field [uav1_pz]
    bufferOffset = _serializer.float64(obj.uav1_pz, buffer, bufferOffset);
    // Serialize message field [uav2_px]
    bufferOffset = _serializer.float64(obj.uav2_px, buffer, bufferOffset);
    // Serialize message field [uav2_py]
    bufferOffset = _serializer.float64(obj.uav2_py, buffer, bufferOffset);
    // Serialize message field [uav2_pz]
    bufferOffset = _serializer.float64(obj.uav2_pz, buffer, bufferOffset);
    // Serialize message field [g0_px]
    bufferOffset = _serializer.float64(obj.g0_px, buffer, bufferOffset);
    // Serialize message field [g0_py]
    bufferOffset = _serializer.float64(obj.g0_py, buffer, bufferOffset);
    // Serialize message field [g0_pz]
    bufferOffset = _serializer.float64(obj.g0_pz, buffer, bufferOffset);
    // Serialize message field [g1_px]
    bufferOffset = _serializer.float64(obj.g1_px, buffer, bufferOffset);
    // Serialize message field [g1_py]
    bufferOffset = _serializer.float64(obj.g1_py, buffer, bufferOffset);
    // Serialize message field [g1_pz]
    bufferOffset = _serializer.float64(obj.g1_pz, buffer, bufferOffset);
    // Serialize message field [g2_px]
    bufferOffset = _serializer.float64(obj.g2_px, buffer, bufferOffset);
    // Serialize message field [g2_py]
    bufferOffset = _serializer.float64(obj.g2_py, buffer, bufferOffset);
    // Serialize message field [g2_pz]
    bufferOffset = _serializer.float64(obj.g2_pz, buffer, bufferOffset);
    // Serialize message field [time]
    bufferOffset = _serializer.float64(obj.time, buffer, bufferOffset);
    // Serialize message field [uav0_pid_vx]
    bufferOffset = _serializer.float64(obj.uav0_pid_vx, buffer, bufferOffset);
    // Serialize message field [uav0_pid_vy]
    bufferOffset = _serializer.float64(obj.uav0_pid_vy, buffer, bufferOffset);
    // Serialize message field [uav0_pid_vz]
    bufferOffset = _serializer.float64(obj.uav0_pid_vz, buffer, bufferOffset);
    // Serialize message field [uav0_aS]
    bufferOffset = _serializer.float64(obj.uav0_aS, buffer, bufferOffset);
    // Serialize message field [uav0_a_vx]
    bufferOffset = _serializer.float64(obj.uav0_a_vx, buffer, bufferOffset);
    // Serialize message field [uav0_a_vy]
    bufferOffset = _serializer.float64(obj.uav0_a_vy, buffer, bufferOffset);
    // Serialize message field [uav0_a_vz]
    bufferOffset = _serializer.float64(obj.uav0_a_vz, buffer, bufferOffset);
    // Serialize message field [uav0_cS]
    bufferOffset = _serializer.float64(obj.uav0_cS, buffer, bufferOffset);
    // Serialize message field [uav0_c_vx]
    bufferOffset = _serializer.float64(obj.uav0_c_vx, buffer, bufferOffset);
    // Serialize message field [uav0_c_vy]
    bufferOffset = _serializer.float64(obj.uav0_c_vy, buffer, bufferOffset);
    // Serialize message field [uav0_c_vz]
    bufferOffset = _serializer.float64(obj.uav0_c_vz, buffer, bufferOffset);
    // Serialize message field [uav1_pid_vx]
    bufferOffset = _serializer.float64(obj.uav1_pid_vx, buffer, bufferOffset);
    // Serialize message field [uav1_pid_vy]
    bufferOffset = _serializer.float64(obj.uav1_pid_vy, buffer, bufferOffset);
    // Serialize message field [uav1_pid_vz]
    bufferOffset = _serializer.float64(obj.uav1_pid_vz, buffer, bufferOffset);
    // Serialize message field [uav1_aS]
    bufferOffset = _serializer.float64(obj.uav1_aS, buffer, bufferOffset);
    // Serialize message field [uav1_a_vx]
    bufferOffset = _serializer.float64(obj.uav1_a_vx, buffer, bufferOffset);
    // Serialize message field [uav1_a_vy]
    bufferOffset = _serializer.float64(obj.uav1_a_vy, buffer, bufferOffset);
    // Serialize message field [uav1_a_vz]
    bufferOffset = _serializer.float64(obj.uav1_a_vz, buffer, bufferOffset);
    // Serialize message field [uav1_cS]
    bufferOffset = _serializer.float64(obj.uav1_cS, buffer, bufferOffset);
    // Serialize message field [uav1_c_vx]
    bufferOffset = _serializer.float64(obj.uav1_c_vx, buffer, bufferOffset);
    // Serialize message field [uav1_c_vy]
    bufferOffset = _serializer.float64(obj.uav1_c_vy, buffer, bufferOffset);
    // Serialize message field [uav1_c_vz]
    bufferOffset = _serializer.float64(obj.uav1_c_vz, buffer, bufferOffset);
    // Serialize message field [uav2_pid_vx]
    bufferOffset = _serializer.float64(obj.uav2_pid_vx, buffer, bufferOffset);
    // Serialize message field [uav2_pid_vy]
    bufferOffset = _serializer.float64(obj.uav2_pid_vy, buffer, bufferOffset);
    // Serialize message field [uav2_pid_vz]
    bufferOffset = _serializer.float64(obj.uav2_pid_vz, buffer, bufferOffset);
    // Serialize message field [uav2_aS]
    bufferOffset = _serializer.float64(obj.uav2_aS, buffer, bufferOffset);
    // Serialize message field [uav2_a_vx]
    bufferOffset = _serializer.float64(obj.uav2_a_vx, buffer, bufferOffset);
    // Serialize message field [uav2_a_vy]
    bufferOffset = _serializer.float64(obj.uav2_a_vy, buffer, bufferOffset);
    // Serialize message field [uav2_a_vz]
    bufferOffset = _serializer.float64(obj.uav2_a_vz, buffer, bufferOffset);
    // Serialize message field [uav2_cS]
    bufferOffset = _serializer.float64(obj.uav2_cS, buffer, bufferOffset);
    // Serialize message field [uav2_c_vx]
    bufferOffset = _serializer.float64(obj.uav2_c_vx, buffer, bufferOffset);
    // Serialize message field [uav2_c_vy]
    bufferOffset = _serializer.float64(obj.uav2_c_vy, buffer, bufferOffset);
    // Serialize message field [uav2_c_vz]
    bufferOffset = _serializer.float64(obj.uav2_c_vz, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type position
    let len;
    let data = new position(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [uav0_px]
    data.uav0_px = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [uav0_py]
    data.uav0_py = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [uav0_pz]
    data.uav0_pz = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [uav1_px]
    data.uav1_px = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [uav1_py]
    data.uav1_py = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [uav1_pz]
    data.uav1_pz = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [uav2_px]
    data.uav2_px = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [uav2_py]
    data.uav2_py = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [uav2_pz]
    data.uav2_pz = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [g0_px]
    data.g0_px = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [g0_py]
    data.g0_py = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [g0_pz]
    data.g0_pz = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [g1_px]
    data.g1_px = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [g1_py]
    data.g1_py = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [g1_pz]
    data.g1_pz = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [g2_px]
    data.g2_px = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [g2_py]
    data.g2_py = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [g2_pz]
    data.g2_pz = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [time]
    data.time = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [uav0_pid_vx]
    data.uav0_pid_vx = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [uav0_pid_vy]
    data.uav0_pid_vy = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [uav0_pid_vz]
    data.uav0_pid_vz = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [uav0_aS]
    data.uav0_aS = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [uav0_a_vx]
    data.uav0_a_vx = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [uav0_a_vy]
    data.uav0_a_vy = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [uav0_a_vz]
    data.uav0_a_vz = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [uav0_cS]
    data.uav0_cS = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [uav0_c_vx]
    data.uav0_c_vx = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [uav0_c_vy]
    data.uav0_c_vy = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [uav0_c_vz]
    data.uav0_c_vz = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [uav1_pid_vx]
    data.uav1_pid_vx = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [uav1_pid_vy]
    data.uav1_pid_vy = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [uav1_pid_vz]
    data.uav1_pid_vz = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [uav1_aS]
    data.uav1_aS = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [uav1_a_vx]
    data.uav1_a_vx = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [uav1_a_vy]
    data.uav1_a_vy = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [uav1_a_vz]
    data.uav1_a_vz = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [uav1_cS]
    data.uav1_cS = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [uav1_c_vx]
    data.uav1_c_vx = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [uav1_c_vy]
    data.uav1_c_vy = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [uav1_c_vz]
    data.uav1_c_vz = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [uav2_pid_vx]
    data.uav2_pid_vx = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [uav2_pid_vy]
    data.uav2_pid_vy = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [uav2_pid_vz]
    data.uav2_pid_vz = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [uav2_aS]
    data.uav2_aS = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [uav2_a_vx]
    data.uav2_a_vx = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [uav2_a_vy]
    data.uav2_a_vy = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [uav2_a_vz]
    data.uav2_a_vz = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [uav2_cS]
    data.uav2_cS = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [uav2_c_vx]
    data.uav2_c_vx = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [uav2_c_vy]
    data.uav2_c_vy = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [uav2_c_vz]
    data.uav2_c_vz = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 416;
  }

  static datatype() {
    // Returns string type for a message object
    return 'practice/position';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'efbb7ed7259937659d1ca1eefb10b7e9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    
    float64 uav0_px
    float64 uav0_py
    float64 uav0_pz
    
    float64 uav1_px
    float64 uav1_py
    float64 uav1_pz
    
    float64 uav2_px
    float64 uav2_py
    float64 uav2_pz
    
    float64 g0_px
    float64 g0_py
    float64 g0_pz
    
    float64 g1_px
    float64 g1_py
    float64 g1_pz
    
    float64 g2_px
    float64 g2_py
    float64 g2_pz
    
    float64 time
    
    float64 uav0_pid_vx
    float64 uav0_pid_vy
    float64 uav0_pid_vz
    
    float64 uav0_aS
    float64 uav0_a_vx
    float64 uav0_a_vy
    float64 uav0_a_vz
    
    float64 uav0_cS
    float64 uav0_c_vx
    float64 uav0_c_vy
    float64 uav0_c_vz
    
    float64 uav1_pid_vx
    float64 uav1_pid_vy
    float64 uav1_pid_vz
    
    float64 uav1_aS
    float64 uav1_a_vx
    float64 uav1_a_vy
    float64 uav1_a_vz
    
    float64 uav1_cS
    float64 uav1_c_vx
    float64 uav1_c_vy
    float64 uav1_c_vz
    
    float64 uav2_pid_vx
    float64 uav2_pid_vy
    float64 uav2_pid_vz
    
    float64 uav2_aS
    float64 uav2_a_vx
    float64 uav2_a_vy
    float64 uav2_a_vz
    
    float64 uav2_cS
    float64 uav2_c_vx
    float64 uav2_c_vy
    float64 uav2_c_vz
    
    
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new position(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.uav0_px !== undefined) {
      resolved.uav0_px = msg.uav0_px;
    }
    else {
      resolved.uav0_px = 0.0
    }

    if (msg.uav0_py !== undefined) {
      resolved.uav0_py = msg.uav0_py;
    }
    else {
      resolved.uav0_py = 0.0
    }

    if (msg.uav0_pz !== undefined) {
      resolved.uav0_pz = msg.uav0_pz;
    }
    else {
      resolved.uav0_pz = 0.0
    }

    if (msg.uav1_px !== undefined) {
      resolved.uav1_px = msg.uav1_px;
    }
    else {
      resolved.uav1_px = 0.0
    }

    if (msg.uav1_py !== undefined) {
      resolved.uav1_py = msg.uav1_py;
    }
    else {
      resolved.uav1_py = 0.0
    }

    if (msg.uav1_pz !== undefined) {
      resolved.uav1_pz = msg.uav1_pz;
    }
    else {
      resolved.uav1_pz = 0.0
    }

    if (msg.uav2_px !== undefined) {
      resolved.uav2_px = msg.uav2_px;
    }
    else {
      resolved.uav2_px = 0.0
    }

    if (msg.uav2_py !== undefined) {
      resolved.uav2_py = msg.uav2_py;
    }
    else {
      resolved.uav2_py = 0.0
    }

    if (msg.uav2_pz !== undefined) {
      resolved.uav2_pz = msg.uav2_pz;
    }
    else {
      resolved.uav2_pz = 0.0
    }

    if (msg.g0_px !== undefined) {
      resolved.g0_px = msg.g0_px;
    }
    else {
      resolved.g0_px = 0.0
    }

    if (msg.g0_py !== undefined) {
      resolved.g0_py = msg.g0_py;
    }
    else {
      resolved.g0_py = 0.0
    }

    if (msg.g0_pz !== undefined) {
      resolved.g0_pz = msg.g0_pz;
    }
    else {
      resolved.g0_pz = 0.0
    }

    if (msg.g1_px !== undefined) {
      resolved.g1_px = msg.g1_px;
    }
    else {
      resolved.g1_px = 0.0
    }

    if (msg.g1_py !== undefined) {
      resolved.g1_py = msg.g1_py;
    }
    else {
      resolved.g1_py = 0.0
    }

    if (msg.g1_pz !== undefined) {
      resolved.g1_pz = msg.g1_pz;
    }
    else {
      resolved.g1_pz = 0.0
    }

    if (msg.g2_px !== undefined) {
      resolved.g2_px = msg.g2_px;
    }
    else {
      resolved.g2_px = 0.0
    }

    if (msg.g2_py !== undefined) {
      resolved.g2_py = msg.g2_py;
    }
    else {
      resolved.g2_py = 0.0
    }

    if (msg.g2_pz !== undefined) {
      resolved.g2_pz = msg.g2_pz;
    }
    else {
      resolved.g2_pz = 0.0
    }

    if (msg.time !== undefined) {
      resolved.time = msg.time;
    }
    else {
      resolved.time = 0.0
    }

    if (msg.uav0_pid_vx !== undefined) {
      resolved.uav0_pid_vx = msg.uav0_pid_vx;
    }
    else {
      resolved.uav0_pid_vx = 0.0
    }

    if (msg.uav0_pid_vy !== undefined) {
      resolved.uav0_pid_vy = msg.uav0_pid_vy;
    }
    else {
      resolved.uav0_pid_vy = 0.0
    }

    if (msg.uav0_pid_vz !== undefined) {
      resolved.uav0_pid_vz = msg.uav0_pid_vz;
    }
    else {
      resolved.uav0_pid_vz = 0.0
    }

    if (msg.uav0_aS !== undefined) {
      resolved.uav0_aS = msg.uav0_aS;
    }
    else {
      resolved.uav0_aS = 0.0
    }

    if (msg.uav0_a_vx !== undefined) {
      resolved.uav0_a_vx = msg.uav0_a_vx;
    }
    else {
      resolved.uav0_a_vx = 0.0
    }

    if (msg.uav0_a_vy !== undefined) {
      resolved.uav0_a_vy = msg.uav0_a_vy;
    }
    else {
      resolved.uav0_a_vy = 0.0
    }

    if (msg.uav0_a_vz !== undefined) {
      resolved.uav0_a_vz = msg.uav0_a_vz;
    }
    else {
      resolved.uav0_a_vz = 0.0
    }

    if (msg.uav0_cS !== undefined) {
      resolved.uav0_cS = msg.uav0_cS;
    }
    else {
      resolved.uav0_cS = 0.0
    }

    if (msg.uav0_c_vx !== undefined) {
      resolved.uav0_c_vx = msg.uav0_c_vx;
    }
    else {
      resolved.uav0_c_vx = 0.0
    }

    if (msg.uav0_c_vy !== undefined) {
      resolved.uav0_c_vy = msg.uav0_c_vy;
    }
    else {
      resolved.uav0_c_vy = 0.0
    }

    if (msg.uav0_c_vz !== undefined) {
      resolved.uav0_c_vz = msg.uav0_c_vz;
    }
    else {
      resolved.uav0_c_vz = 0.0
    }

    if (msg.uav1_pid_vx !== undefined) {
      resolved.uav1_pid_vx = msg.uav1_pid_vx;
    }
    else {
      resolved.uav1_pid_vx = 0.0
    }

    if (msg.uav1_pid_vy !== undefined) {
      resolved.uav1_pid_vy = msg.uav1_pid_vy;
    }
    else {
      resolved.uav1_pid_vy = 0.0
    }

    if (msg.uav1_pid_vz !== undefined) {
      resolved.uav1_pid_vz = msg.uav1_pid_vz;
    }
    else {
      resolved.uav1_pid_vz = 0.0
    }

    if (msg.uav1_aS !== undefined) {
      resolved.uav1_aS = msg.uav1_aS;
    }
    else {
      resolved.uav1_aS = 0.0
    }

    if (msg.uav1_a_vx !== undefined) {
      resolved.uav1_a_vx = msg.uav1_a_vx;
    }
    else {
      resolved.uav1_a_vx = 0.0
    }

    if (msg.uav1_a_vy !== undefined) {
      resolved.uav1_a_vy = msg.uav1_a_vy;
    }
    else {
      resolved.uav1_a_vy = 0.0
    }

    if (msg.uav1_a_vz !== undefined) {
      resolved.uav1_a_vz = msg.uav1_a_vz;
    }
    else {
      resolved.uav1_a_vz = 0.0
    }

    if (msg.uav1_cS !== undefined) {
      resolved.uav1_cS = msg.uav1_cS;
    }
    else {
      resolved.uav1_cS = 0.0
    }

    if (msg.uav1_c_vx !== undefined) {
      resolved.uav1_c_vx = msg.uav1_c_vx;
    }
    else {
      resolved.uav1_c_vx = 0.0
    }

    if (msg.uav1_c_vy !== undefined) {
      resolved.uav1_c_vy = msg.uav1_c_vy;
    }
    else {
      resolved.uav1_c_vy = 0.0
    }

    if (msg.uav1_c_vz !== undefined) {
      resolved.uav1_c_vz = msg.uav1_c_vz;
    }
    else {
      resolved.uav1_c_vz = 0.0
    }

    if (msg.uav2_pid_vx !== undefined) {
      resolved.uav2_pid_vx = msg.uav2_pid_vx;
    }
    else {
      resolved.uav2_pid_vx = 0.0
    }

    if (msg.uav2_pid_vy !== undefined) {
      resolved.uav2_pid_vy = msg.uav2_pid_vy;
    }
    else {
      resolved.uav2_pid_vy = 0.0
    }

    if (msg.uav2_pid_vz !== undefined) {
      resolved.uav2_pid_vz = msg.uav2_pid_vz;
    }
    else {
      resolved.uav2_pid_vz = 0.0
    }

    if (msg.uav2_aS !== undefined) {
      resolved.uav2_aS = msg.uav2_aS;
    }
    else {
      resolved.uav2_aS = 0.0
    }

    if (msg.uav2_a_vx !== undefined) {
      resolved.uav2_a_vx = msg.uav2_a_vx;
    }
    else {
      resolved.uav2_a_vx = 0.0
    }

    if (msg.uav2_a_vy !== undefined) {
      resolved.uav2_a_vy = msg.uav2_a_vy;
    }
    else {
      resolved.uav2_a_vy = 0.0
    }

    if (msg.uav2_a_vz !== undefined) {
      resolved.uav2_a_vz = msg.uav2_a_vz;
    }
    else {
      resolved.uav2_a_vz = 0.0
    }

    if (msg.uav2_cS !== undefined) {
      resolved.uav2_cS = msg.uav2_cS;
    }
    else {
      resolved.uav2_cS = 0.0
    }

    if (msg.uav2_c_vx !== undefined) {
      resolved.uav2_c_vx = msg.uav2_c_vx;
    }
    else {
      resolved.uav2_c_vx = 0.0
    }

    if (msg.uav2_c_vy !== undefined) {
      resolved.uav2_c_vy = msg.uav2_c_vy;
    }
    else {
      resolved.uav2_c_vy = 0.0
    }

    if (msg.uav2_c_vz !== undefined) {
      resolved.uav2_c_vz = msg.uav2_c_vz;
    }
    else {
      resolved.uav2_c_vz = 0.0
    }

    return resolved;
    }
};

module.exports = position;
