//
//  OrientaionModel.swift
//  3DOrientation
//
//  Created by Joshua Yang on 2023/3/20.
//

import Foundation
import SwiftUI
import CoreMotion
import MultipeerConnectivity

struct OrientationData: Codable {
    var yaw: Double
    var pitch: Double
    var roll: Double
    var rotateMatrix: Matrix
    
    init() {
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.rotateMatrix = Matrix(identityOfSize: 3)
    }
}

class OrientationModel: ObservableObject {
    
    var manager: CMMotionManager
    var kalmanFilter: KalmanFilter
    var gyroQueue: CMRotationRate?
    var orientationSession: OrientationSession
    
    
    init() {
        self.manager = CMMotionManager()
        self.kalmanFilter = KalmanFilter()
        self.orientationSession = OrientationSession()
        
        manager.deviceMotionUpdateInterval = 0.1
        manager.startDeviceMotionUpdates(using: CMAttitudeReferenceFrame.xMagneticNorthZVertical, to: .main) { (motion, error) in
            if let previousGyroData = self.gyroQueue {
                self.gyroQueue = motion!.rotationRate
                if self.orientationSession.paired {
                    let A = motion!.gravity
                    let M = motion!.magneticField.field
                    let U = Matrix(vector: [previousGyroData.x, previousGyroData.y, previousGyroData.z])
                    self.kalmanFilter.predict(U: U, dt: 0.1)
                    self.kalmanFilter.update(A: Matrix(vector: [A.x, A.y, A.z]), M: Matrix(vector: [M.x, M.y, M.z]))
                    //                print("Estiamte: ", terminator: "")
                    //                print(self.getEulerAngles(q: self.kalmanFilter.posterioriX))
                    //                print("Measured: ", terminator: "")
                    //                print(motion!.attitude)
                    self.orientationSession.sendData(data: self.getEulerAngles(q: self.kalmanFilter.posterioriX))
                    print("Yaw: \(self.getEulerAngles(q: self.kalmanFilter.posterioriX).yaw),  Pitch: \(self.getEulerAngles(q: self.kalmanFilter.posterioriX).pitch), Roll: \(self.getEulerAngles(q: self.kalmanFilter.posterioriX).roll)")
                }
            }
            else {
                self.gyroQueue = motion!.rotationRate
            }
        }
    }

    func rad2deg(rad: Double) -> Double {
        rad * 180 / .pi
    }
    
    func deg2rad(deg: Double) -> Double {
        deg * 180 /  .pi
    }
    
    func getRotMat(q: Matrix) -> Matrix {
        let c00 = pow(q[0, 0], 2) + pow(q[1, 0], 2) - pow(q[2, 0], 2) - pow(q[3, 0], 2)
        let c01 = 2 * (q[1, 0] * q[2, 0] - q[0, 0] * q[3, 0])
        let c02 = 2 * (q[1, 0] * q[3, 0] + q[0, 0] * q[2, 0])
        let c10 = 2 * (q[1, 0] * q[2, 0] + q[0, 0] * q[3, 0])
        let c11 = pow(q[0, 0], 2) - pow(q[1, 0], 2) + pow(q[2, 0], 2) - pow(q[3, 0], 2)
        let c12 = 2 * (q[2, 0] * q[3, 0] - q[0, 0] * q[1, 0])
        let c20 = 2 * (q[1, 0] * q[3, 0] - q[0, 0] * q[2, 0])
        let c21 = 2 * (q[2, 0] * q[3, 0] + q[0, 0] * q[1, 0])
        let c22 = pow(q[0, 0], 2) - pow(q[1, 0], 2) - pow(q[2, 0], 2) + pow(q[3, 0], 2)
        
        return Matrix([[c00, c01, c02], [c10, c11, c12], [c20, c21, c22]])
    }
    
    func getEulerAngles(q: Matrix) -> OrientationData {
        var angles = OrientationData()
        var roll: Double
        var pitch: Double
        var yaw: Double
        
        let w = q[0, 0]
        let x = q[1, 0]
        let y = q[2, 0]
        let z = q[3, 0]
        let t0 = 2.0 * (w * x + y * z)
        let t1 = 1.0 - 2.0 * (x * x + y * y)
        pitch = atan2(t0, t1)
        
        var t2 = 2.0 * (w * y - z * x)
        if t2 > 1.0 {
            t2 = 1.0
        }
        else if t2 < -1.0 {
            t2 = -1.0
        }
        roll = asin(t2)
        
        let t3 = 2.0 * (w * z + x * y)
        let t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = atan2(t3, t4)
        
        angles.yaw = rad2deg(rad: yaw)
        angles.pitch = rad2deg(rad: pitch)
        angles.roll = rad2deg(rad: roll)
        angles.rotateMatrix = getRotMat(q: q)
        
        return angles
    }
    

}


