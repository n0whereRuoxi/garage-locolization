//
//  DataProcessor.swift
//  ACC
//
//  Created by Hung-Yun Liao on 6/29/16.
//  Copyright © 2016 Hung-Yun Liao. All rights reserved.
//

import Foundation
import CoreMotion
import HCKalmanFilter
import CoreLocation

protocol DataProcessorDelegate {
    func sendingNewData(person: DataProcessor, type: speedDataType, data: ThreeAxesSystemDouble)
    func sendingNewStatus(person: DataProcessor, status: String)
    func sendingFloorChangeSourceData(source: FloorChangeSource, val: Double)
    func sendingFloorChange(source: FloorChangeSource, change: Int)
}

enum FloorChangeSource {
    case altitude
    case acceleration
    case rotation
}

enum speedDataType {
    case accelerate
    case velocity
    case distance
    case rotation
}

class DataProcessor {
    
    // MARK: delegate
    var delegate: DataProcessorDelegate? = nil
    
    func newData(type: speedDataType, sensorData: ThreeAxesSystemDouble) {
        delegate?.sendingNewData(person: self, type: type, data: sensorData)
    }
    
    func newStatus(status: String) {
        delegate?.sendingNewStatus(person: self, status: status)
    }
    
    // MARK: System parameters setup
    let gravityConstant = 9.81457
    var deviceMotionUpdateInterval: Double = 0.01
    let accelerationThreshold = 0.0
    
    // MARK: Instance variables
    var motionManager = CMMotionManager()
    var absSys: System = System()

    // MARK: motion activity manager
    let motionActivityManager = CMMotionActivityManager()
    var is_stationary = true
    
    // MARK: floor change from altitude
    let altimeter = CMAltimeter()
    let floor_height_lowerbound = 3.0
    var cur_altitude = 0.0
    var last_floor_altitude = 0.0
    
    // MARK: floor change from acceleration
    let velocity_z_upperbound = 5.0
    let velocity_z_lowerbound = 0.1
    var velocity_pending_floor_change = 0
    
    // MARK: floor change from rotation
    var initial_pitch: Double?
    let pitch_upperbound = 20.0 * Double.pi / 180
    let pitch_lowerbound = 5.0 * Double.pi / 180
    var pitch_pending_floor_change = 0
    
    var pre_ts:Double? = nil
    
    func startsDetection() {
        
        // Set Motion Manager Properties
        motionManager.deviceMotionUpdateInterval = deviceMotionUpdateInterval
        
        motionManager.startDeviceMotionUpdates(using: CMAttitudeReferenceFrame.xTrueNorthZVertical , to: OperationQueue.current!, withHandler: { (motion,  error) in
            if motion != nil {
                self.trueNorth(motion: motion!)
            }
            if error != nil {
                print("\(String(describing: error))")
            }
        })
//        motionManager.deviceMotion.at
        
        motionActivityManager.startActivityUpdates(to: OperationQueue.current!) { (motion) in
            if motion?.stationary == false {
                self.is_stationary = false
            } else {
                self.is_stationary = true
            }
        }
        
        altimeter.startRelativeAltitudeUpdates(to: OperationQueue.current!) { (data, error) in
            if let relative_altitude = data?.relativeAltitude {
                self.cur_altitude = Double(truncating: relative_altitude)
                self.delegate?.sendingFloorChangeSourceData(source: .altitude, val: self.cur_altitude)
            }
        }
    }
    
    func reset() {
        absSys.reset()
    }
    
    func resetWith(vx:Double, vy:Double, vz: Double) {
        absSys.distance.x = 0
        absSys.distance.y = 0
        absSys.distance.z = 0
        absSys.velocity.x = vx
        absSys.velocity.y = vy
        absSys.velocity.z = vz
    }
    
    // MARK: Functions
    func trueNorth(motion: CMDeviceMotion) {
        
        let acc: CMAcceleration = motion.userAcceleration
        //print(acc)
        let rot = motion.attitude.rotationMatrix
        if initial_pitch == nil {
            initial_pitch = motion.attitude.pitch
        }
        let pitch = motion.attitude.pitch - initial_pitch!
        delegate?.sendingFloorChangeSourceData(source: .rotation, val: pitch)
        if  fabs(pitch) > pitch_upperbound {
            pitch_pending_floor_change = pitch > 0 ? 1 : -1
        }
        if fabs(pitch) <  pitch_lowerbound && pitch_pending_floor_change != 0 && fabs(cur_altitude - last_floor_altitude) > floor_height_lowerbound {
            delegate?.sendingFloorChange(source: .rotation, change: pitch_pending_floor_change)
            last_floor_altitude = cur_altitude
            pitch_pending_floor_change = 0
        }
        
        let x = (acc.x*rot.m11 + acc.y*rot.m21 + acc.z*rot.m31) * gravityConstant
        let y = (acc.x*rot.m12 + acc.y*rot.m22 + acc.z*rot.m32) * gravityConstant
        let z = (acc.x*rot.m13 + acc.y*rot.m23 + acc.z*rot.m33) * gravityConstant
        
        (absSys.accelerate.x, absSys.accelerate.y, absSys.accelerate.z) = (x, y, z)
        
        let cur_ts = CFAbsoluteTimeGetCurrent()
        let dt = pre_ts == nil ? 0 : cur_ts - pre_ts!
        determineVelocityAndCoculateDistance(delta_ts: dt)
        pre_ts = cur_ts
        
        newData(type: speedDataType.accelerate, sensorData: absSys.accelerate)
        newData(type: speedDataType.velocity, sensorData: absSys.velocity)
        newData(type: speedDataType.distance, sensorData: absSys.distance)
        
        
        // Floor change from acceleration
//        delegate?.sendingFloorChangeSourceData(source: .acceleration, val: absSys.velocity.z)
//
//        if fabs(absSys.velocity.z) > velocity_z_upperbound {
//            velocity_pending_floor_change = absSys.velocity.z > 0 ? 1 : -1
//        }
//
//        if fabs(absSys.velocity.z) < velocity_z_lowerbound && velocity_pending_floor_change != 0 {
//            delegate?.sendingFloorChange(source: .acceleration, change: velocity_pending_floor_change)
//            absSys.velocity.z = 0
//            velocity_pending_floor_change = 0
//        }
        
//        absSys.accelerate.x = 0
//        absSys.accelerate.y = 0
//        absSys.accelerate.z = 0
    }
    
    func determineVelocityAndCoculateDistance(delta_ts: Double) {
        if is_stationary {
            newStatus(status: "static state") // sending status to delegate
            absSys.accelerate.x = 0
            absSys.accelerate.y = 0
            absSys.accelerate.z = 0
            absSys.velocity.x = 0
            absSys.velocity.y = 0
            absSys.velocity.z = 0

        } else {
            newStatus(status: "dynamic state") // sending status to delegate
            let vx = absSys.velocity.x
            let vy = absSys.velocity.y
            let vz = absSys.velocity.z
            if fabs(absSys.accelerate.x) > accelerationThreshold {
                absSys.velocity.x += absSys.accelerate.x * delta_ts
            }
            absSys.distance.x += (vx + absSys.velocity.x) * delta_ts / 2
            if fabs(absSys.accelerate.y) > accelerationThreshold {
                absSys.velocity.y += absSys.accelerate.y * delta_ts
            }
            absSys.distance.y += (vy + absSys.velocity.y) * delta_ts / 2
            if fabs(absSys.accelerate.z) > accelerationThreshold {
                absSys.velocity.z += absSys.accelerate.z * delta_ts
            }
            absSys.distance.z += (vz + absSys.velocity.z) * delta_ts / 2
        }
    }
}


