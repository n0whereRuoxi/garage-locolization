//
//  ViewController.swift
//  map
//
//  Created by Ruoxi Li on 4/13/19.
//  Copyright © 2019 Ruoxi Li. All rights reserved.
//

import UIKit
import MapKit
import CoreLocation
import Darwin
import HCKalmanFilter

class ViewController: UIViewController, DataProcessorDelegate {

    @IBOutlet weak var vLabel: UILabel!
    @IBOutlet weak var rotLabel: UILabel!
    @IBOutlet weak var altLabel: UILabel!
    @IBOutlet weak var accLabel: UILabel!
    @IBOutlet weak var floorLabel: UILabel!
    @IBOutlet weak var accuLabel: UILabel!
    @IBOutlet weak var mapView: MKMapView!
    
    
    @IBAction func clearButton(_ sender: UIButton) {
        print("clicked")
        clearTrajectory()
        dataSource.reset()
        // resetKalmanFilter = false
        if myLocation != nil && hcKalmanFilter != nil {
            hcKalmanFilter!.resetKalman(newStartLocation: myLocation!)
        }
    }
    
    var dataSource: DataProcessor = DataProcessor()
    let GPSAccThreshold = 20.0
    var resetKalmanFilter: Bool = false
    var hcKalmanFilter: HCKalmanAlgorithm?
    var locationManager = CLLocationManager()
    var timer: Timer!
    var GPSAnnotation: MKAnnotation? = nil
    var DeadReckoningAnnotation: MKAnnotation? = nil
    var trajectoryAnnotations: [MKAnnotation] = []
    var myLocation: CLLocation? = nil
    var lastKalmanLocation: CLLocation? = nil
    var floor: Int = 0
    
    override func viewDidLoad() {
        super.viewDidLoad()
        mapView.delegate = self
        // Do any additional setup after loading the view.
        mapView.showsUserLocation = true
        mapView.userTrackingMode = .follow
        if CLLocationManager.locationServicesEnabled() == true {
            if CLLocationManager.authorizationStatus() == .restricted ||
                CLLocationManager.authorizationStatus() == .denied ||
                CLLocationManager.authorizationStatus() == .notDetermined {
                locationManager.requestWhenInUseAuthorization()
            }
            locationManager.desiredAccuracy = kCLLocationAccuracyBest
            locationManager.delegate = self
            locationManager.startUpdatingLocation()
        } else {
            print("turn on GPS")
        }
        
        dataSource.startsDetection()
        
        //timer = Timer.scheduledTimer(timeInterval: 0.1, target: self, selector: #selector(ViewController.updateDeadReckoning), userInfo: nil, repeats: true)
        //print("end")
        
        
    }

    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        NotificationCenter.default.post(name: NSNotification.Name(rawValue: "dataSource"), object: dataSource)
    }
    
    override func viewDidAppear(_ animated: Bool) {
        super.viewDidAppear(animated)
        dataSource.delegate = self
    }
    
    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // Dispose of any resources that can be recreated.
    }
    
    @objc func updateDeadReckoning() {
//        if myLocation != nil {
//            if hcKalmanFilter == nil {
//                self.hcKalmanFilter = HCKalmanAlgorithm(initialLocation: myLocation!)
//            }
//            else {
//                if let hcKalmanFilter = self.hcKalmanFilter {
//                    if resetKalmanFilter == true {
//                        hcKalmanFilter.resetKalman(newStartLocation: myLocation!)
//                        resetKalmanFilter = false
//                    }
//                    else {
//                        let kalmanLocation = hcKalmanFilter.processState(currentLocation: myLocation!)
//                        print(kalmanLocation.coordinate)
//                        myLocation = kalmanLocation
//                        updateAnnotation("DeadReckoning", kalmanLocation.coordinate)
//                    }
//                }
//            }
//        }
    }
    
    func clearTrajectory() {
        for annotation in trajectoryAnnotations {
            mapView.removeAnnotation(annotation)
        }
        trajectoryAnnotations = []
    }
    
    func addTrajectoryAnnotation(_ latitude: Double, _ longitude: Double) {
        let annotation = MKPointAnnotation()
        annotation.title = "Trajectory"
        annotation.coordinate = CLLocationCoordinate2D(latitude: latitude, longitude: longitude)
        trajectoryAnnotations.append(annotation)
        mapView.addAnnotation(annotation)
    }
    
    func sendingNewData(person: DataProcessor, type: speedDataType, data: ThreeAxesSystemDouble) {
        if type == .distance {
            let rEarth = 6371000.0
            if lastKalmanLocation != nil {
                let newLatitude  = lastKalmanLocation!.coordinate.latitude  + (data.x / rEarth) * (180 / Double.pi);
                let newLongitude = lastKalmanLocation!.coordinate.longitude + (data.y / rEarth) * (180 / Double.pi) / cos(lastKalmanLocation!.coordinate.latitude * Double.pi/180);
                // print("kalman location", lastKalmanLocation!.coordinate.latitude, lastKalmanLocation!.coordinate.longitude)
                // print("dead reckoning location", newLatitude, newLongitude)
                //addTrajectoryAnnotation(newLatitude, newLongitude)
                updateAnnotation("DeadReckoning", CLLocationCoordinate2D(latitude: newLatitude, longitude: newLongitude))
            }
        }
    }
    
    func sendingAltitude(data: Double) {
        //print("Altutude: ", data)
    }
    
    func sendingFloorChangeSourceData(source: FloorChangeSource, val: Double) {
        switch source {
        case .acceleration:
            break
        case .altitude:
            altLabel.text = "Altitude: " + String(val).prefix(4) + " m"
        case .rotation:
            rotLabel.text = "Rotation: " + String(val / Double.pi * 180).prefix(4) + " °"
        }
    }
    
    func sendingFloorChange(source: FloorChangeSource, change: Int) {
        if source == .rotation {
            print("By rotation: ", change)
            floor += change
            floorLabel.text = "Floor: " + String(floor)
        }
    }
    
    func sendingNewStatus(person: DataProcessor, status: String) {}
    
    func updateAnnotation(_ title: String, _ coordinate: CLLocationCoordinate2D) {
        let annotation = MKPointAnnotation()
        annotation.coordinate = coordinate
        if title == "GPS" {
            annotation.title = "GPS"
            mapView.addAnnotation(annotation)
            if GPSAnnotation != nil {
                mapView.removeAnnotation(GPSAnnotation!)
            }
            GPSAnnotation = annotation
        } else if title == "DeadReckoning" {
            annotation.title = "DeadReckoning"
            mapView.addAnnotation(annotation)
            if DeadReckoningAnnotation != nil {
                mapView.removeAnnotation(DeadReckoningAnnotation!)
            }
            DeadReckoningAnnotation = annotation
        }
    }
}

extension ViewController: MKMapViewDelegate {
    func mapView(_ mapView: MKMapView, viewFor annotation: MKAnnotation) -> MKAnnotationView? {
        var annotationView = mapView.dequeueReusableAnnotationView(withIdentifier: "AnnotationView")
        if annotationView == nil {
            annotationView = MKAnnotationView(annotation: annotation, reuseIdentifier: "AnnotationView")
        }
        if annotation.title == "GPS" {
            annotationView?.image = UIImage(named: "red")
        } else if annotation.title == "DeadReckoning" {
            annotationView?.image = UIImage(named: "blue")
        } else if annotation.title == "Trajectory" {
            annotationView?.image = UIImage(named: "dot")
        }
        return annotationView
    }
}

extension ViewController: CLLocationManagerDelegate {

    //MARK:- CLLocationManager Delegates
    
    func locationManager(_ manager: CLLocationManager, didUpdateLocations locations: [CLLocation]) {
        
        accuLabel.text = "GPS Accuracy: " + String(locations[0].horizontalAccuracy) + " m"
        
//        let region = MKCoordinateRegion(center: CLLocationCoordinate2D(latitude: locations[0].coordinate.latitude, longitude: locations[0].coordinate.longitude), span: MKCoordinateSpan(latitudeDelta: 0.001, longitudeDelta: 0.001))
//        self.mapView.setRegion(region, animated: true)
//        if locations[0].floor != nil {
//            floorLabel.text = "\(locations[0].floor!.level)"
//        }
        //        print("speed", locations[0].speed)
//        if myLocation!.horizontalAccuracy < GPSAccThreshold {
//            myLocation = locations.first!
//            let region = MKCoordinateRegion(center: CLLocationCoordinate2D(latitude: locations[0].coordinate.latitude, longitude: locations[0].coordinate.longitude), span: MKCoordinateSpan(latitudeDelta: 0.0001, longitudeDelta: 0.0001))
//            self.mapView.setRegion(region, animated: true)
//            updateAnnotation("GPS", myLocation!.coordinate)
//            updateAnnotation("DeadReckoning", myLocation!.coordinate)
//            resetKalmanFilter = true
//        }

        let myLocation: CLLocation = locations.first!
        
        if hcKalmanFilter == nil {
            self.hcKalmanFilter = HCKalmanAlgorithm(initialLocation: myLocation)
        }
        else {
            if let hcKalmanFilter = self.hcKalmanFilter {
                var (kalmanLocation, vlat, vlong, vz) = hcKalmanFilter.processState(currentLocation: myLocation)
                updateAnnotation("GPS", myLocation.coordinate)
                updateAnnotation("DeadReckoning", kalmanLocation.coordinate)
//                print(vlat, vlong)
                let rEarth = 6371000.0
                var vx = vlat / (180 / Double.pi) * rEarth
                var vy = vlong * cos(kalmanLocation.coordinate.latitude * Double.pi/180) / (180 / Double.pi) * rEarth
                if vx.isNaN || vy.isNaN {
                    vx = 0.0
                    vy = 0.0
                    vz = 0.0
                }
                vLabel.text = "Speed: " + String(2.23694 * sqrt(pow(vx, 2) + pow(vy, 2) + pow(vz, 2))).prefix(4) + " mph"
                dataSource.resetWith(vx: vx, vy: vy, vz: vz)
                lastKalmanLocation = kalmanLocation
            }
        }
    }

    func locationManager(_ manager: CLLocationManager, didFailWithError error: Error) {}

}
