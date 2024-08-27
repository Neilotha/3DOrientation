//
//  KalmanFilter.swift
//  3DOrientation
//
//  Created by Joshua Yang on 2023/3/22.
//

import Foundation

struct KalmanFilter {
    var prioriX: Matrix
    var posterioriX: Matrix
    var prioriStateErrorCovariance: Matrix
    var posterioriStateErrorCovariance: Matrix
    var Q: Matrix
    var R: Matrix
    var K: Matrix?
    var B: Matrix?
    var H: Matrix?
    var accelReference: Matrix
    var magReference: Matrix
    
    init() {
        self.prioriX = Matrix(vector: [1, 0, 0, 0])
        self.posterioriX = Matrix(vector: [1, 0, 0, 0])
        self.prioriStateErrorCovariance = Matrix(identityOfSize: 4)
        self.posterioriStateErrorCovariance = Matrix(identityOfSize: 4)
        self.Q = Matrix(identityOfSize: 4) * 0.001
        self.R = Matrix(identityOfSize: 6) * 0.001
        self.K = nil
        self.B = nil
        self.H = nil
        self.accelReference = Matrix(vector: [0, 0, -1])
        self.magReference = Matrix(vector: [1, 0, 0])
    }
    
    
    func normalizeVector(V: Matrix) -> Matrix {
        var mag = 0.0
        for i in 0..<V.rows {
            mag += pow(V[i, 0], 2)
        }
        
        
        return (1 / mag.squareRoot()) * V
    }
    
    func estimateY() -> Matrix {
        let q = self.prioriX
        /*
        var AHat = Matrix(vector: [2 * (q[1, 0] * q[3, 0] - q[0, 0] * q[2, 0]),
                                        2 * (q[2, 0] * q[3, 0] + q[0,0] * q[1, 0]),
                                        pow(q[0, 0], 2) - pow(q[1, 0], 2) - pow(q[2, 0], 2) + pow(q[3, 0], 2)])
        
        var MHat = Matrix(vector: [2 * (q[1, 0] * q[2, 0] + q[0, 0] * q[3, 0]),
                                        pow(q[0, 0], 2) - pow(q[1, 0], 2) + pow(q[2, 0], 2) - pow(q[3, 0], 2),
                                        2 * (q[2, 0] * q[3, 0] - q[0,0] * q[1, 0])])
        */
        let rotateMat = getRotMat(q: q)
        let AHat = rotateMat.transposed * accelReference
        let MHat = rotateMat.transposed * magReference

        return Matrix(vector: [AHat[0, 0], AHat[1, 0], AHat[2, 0], MHat[0, 0], MHat[1, 0], MHat[2, 0]])
    }
    
    func predictY() -> Matrix {
        let q = self.posterioriX
        /*
        var AHat = Matrix(vector: [2 * (q[1, 0] * q[3, 0] - q[0, 0] * q[2, 0]),
                                        2 * (q[2, 0] * q[3, 0] + q[0,0] * q[1, 0]),
                                        pow(q[0, 0], 2) - pow(q[1, 0], 2) - pow(q[2, 0], 2) + pow(q[3, 0], 2)])
        
        var MHat = Matrix(vector: [2 * (q[1, 0] * q[2, 0] + q[0, 0] * q[3, 0]),
                                        pow(q[0, 0], 2) - pow(q[1, 0], 2) + pow(q[2, 0], 2) - pow(q[3, 0], 2),
                                        2 * (q[2, 0] * q[3, 0] - q[0,0] * q[1, 0])])
        */
        let rotateMat = getRotMat(q: q)
        let AHat = rotateMat.transposed * accelReference
        let MHat = rotateMat.transposed * magReference

        return Matrix(vector: [AHat[0, 0], AHat[1, 0], AHat[2, 0], MHat[0, 0], MHat[1, 0], MHat[2, 0]])
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
    
    mutating func getJacobian(ref: Matrix) -> Matrix {
        /*
        H = Matrix([[-prioriX[2, 0], prioriX[3, 0], prioriX[0, 0], prioriX[1, 0]],
                    [prioriX[1, 0], prioriX[0, 0], prioriX[3, 0], prioriX[2, 0]],
                    [prioriX[0, 0], -prioriX[1, 0], -prioriX[2, 0], prioriX[3, 0]],
                    [prioriX[3, 0], prioriX[2, 0], prioriX[1, 0], prioriX[0, 0]],
                    [prioriX[0, 0], -prioriX[1, 0], prioriX[2, 0], -prioriX[3, 0]],
                    [-prioriX[1, 0], -prioriX[0, 0], prioriX[3, 0], prioriX[2, 0]]])
        self.H = -2 * self.H!
        */
        let e00 = prioriX[0, 0] * ref[0, 0] + prioriX[3, 0] * ref[1, 0] - prioriX[2, 0] * ref[2, 0]
        let e01 = prioriX[1, 0] * ref[0, 0] + prioriX[2, 0] * ref[1, 0] + prioriX[3, 0] * ref[2, 0]
        let e02 = -prioriX[2, 0] * ref[0, 0] + prioriX[1, 0] * ref[1, 0] - prioriX[0, 0] * ref[2, 0]
        let e03 = -prioriX[3, 0] * ref[0, 0] + prioriX[0, 0] * ref[1, 0] + prioriX[1, 0] * ref[2, 0]
        let e10 = -prioriX[3, 0] * ref[0, 0] + prioriX[0, 0] * ref[1, 0] + prioriX[1, 0] * ref[2, 0]
        let e11 = prioriX[2, 0] * ref[0, 0] - prioriX[1, 0] * ref[1, 0] + prioriX[0, 0] * ref[2, 0]
        let e12 = prioriX[1, 0] * ref[0, 0] + prioriX[2, 0] * ref[1, 0] + prioriX[3, 0] * ref[2, 0]
        let e13 = -prioriX[0, 0] * ref[0, 0] - prioriX[3, 0] * ref[1, 0] + prioriX[2, 0] * ref[2, 0]
        let e20 = prioriX[2, 0] * ref[0, 0] - prioriX[1, 0] * ref[1, 0] + prioriX[0, 0] * ref[2, 0]
        let e21 = prioriX[3, 0] * ref[0, 0] - prioriX[0, 0] * ref[1, 0] - prioriX[1, 0] * ref[2, 0]
        let e22 = prioriX[0, 0] * ref[0, 0] + prioriX[3, 0] * ref[1, 0] - prioriX[2, 0] * ref[2, 0]
        let e23 = prioriX[1, 0] * ref[0, 0] + prioriX[2, 0] * ref[1, 0] + prioriX[3, 0] * ref[2, 0]
        
        let jacobian = 2 * Matrix([[e00, e01, e02, e03], [e10, e11, e12, e13], [e20, e21, e22, e23]])
        
        return jacobian
    }
    
    mutating func predict(U: Matrix, dt: Double) {
        let Sq = Matrix([[-posterioriX[1, 0], -posterioriX[2, 0], -posterioriX[3, 0]],
                        [posterioriX[0, 0], -posterioriX[3, 0], posterioriX[2, 0]],
                        [posterioriX[3, 0], posterioriX[0, 0], -posterioriX[1, 0]],
                        [-posterioriX[2, 0], posterioriX[1, 0], posterioriX[0, 0]]])
        let B = (dt / 2) * Sq
        prioriX = normalizeVector(V: (posterioriX + (B * U)))
        let hA = getJacobian(ref: self.accelReference)
        let hM = getJacobian(ref: self.magReference)
        self.H = Matrix([[hA[0, 0], hA[0, 1], hA[0, 2], hA[0, 3]],
                         [hA[1, 0], hA[1, 1], hA[1, 2], hA[1, 3]],
                         [hA[2, 0], hA[2, 1], hA[2, 2], hA[2, 3]],
                         [hM[0, 0], hM[0, 1], hM[0, 2], hM[0, 3]],
                         [hM[1, 0], hM[1, 1], hM[1, 2], hM[1, 3]],
                         [hM[2, 0], hM[2, 1], hM[2, 2], hM[2, 3]],
                        ])
    }
    
    mutating func update(A: Matrix, M: Matrix) {
        K = prioriStateErrorCovariance * H!.transposed *
            (H! * prioriStateErrorCovariance * H!.transposed + R).inversed
        posterioriStateErrorCovariance = (Matrix(identityOfSize: 4) - K! * H!) * prioriStateErrorCovariance
        prioriStateErrorCovariance = posterioriStateErrorCovariance + Q
        
        let a = A
        let m = normalizeVector(V: M)
        let measurement = Matrix(vector: [a[0, 0], a[1, 0], a[2, 0], m[0, 0], m[1, 0], m[2, 0]])
        let yHat = estimateY()
        
        posterioriX = prioriX + K! * (measurement - yHat)
        posterioriX = normalizeVector(V: posterioriX)
//        print("predict:")
//        print(predictY())
//        print("measurement:")
//        print(measurement)
    }
    
    
    
}
