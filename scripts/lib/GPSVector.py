#!/usr/bin/env python

#lib by fgg for EagleX
#Fabian Gomez Gonzalez
#a01209914@itesm.mx

from math import *

#constants for math functions
earthRadius = 6371e3

#---------------------GPSPoint---------------------------------------------------------------------------------------------------------------------------

class GPSPoint:
    #GPSPoint is instantiated using latitude and longitude in decimal degrees notation
    def __init__(self, latitude=0.0, longitude=0.0):
    
        #Handle ValueError exceptions when args are not instantiated correctly
        if not isinstance(float(latitude), float) or latitude < -90.0 or latitude > 90.0:
            raise ValueError('latitude must be a number and has to be between -90.0 and 90.0')
        if not isinstance(float(longitude), float) or longitude < -180.0 or longitude > 180.0:
            raise ValueError('longitude must be a number and has to be between -180.0 and 180.0')
        
        self.latitude  = latitude
        self.longitude = longitude
        
    def __str__(self):
        msg = '[%.6f, %.6f]' % (self.latitude, self.longitude)
        return msg
     
    #distance to GPSPoint
    #returns the shortest distance along the great circle path in meters between 2 GPSPoints using the haversine function
    #source https://www.movable-type.co.uk/scripts/latlong.html
    def distanceToGPSPoint(self, point):
    
        #Handle TypeError exceptions when args are not instantiated correctly
        if not isinstance(point, GPSPoint):
            raise TypeError('distanceToGPSPoint(self,point) must be passed a GPSPoint class object')
            
        phi1        = radians(self.latitude)
        phi2        = radians(point.latitude)
        d_phi       = radians(point.latitude - self.latitude)
        d_lambda    = radians(point.longitude - self.longitude)
        
        a = sin(d_phi/2.0) * sin(d_phi/2.0) + cos(phi1) * cos(phi2) * sin(d_lambda/2.0) * sin(d_lambda/2.0)
        c = 2.0 * atan2(sqrt(a),sqrt(1-a))
        
        d = earthRadius * c
        return d
        
    #initialBearingToGPSPoint
    #returns the initial bearing from the initial point that points to the end point assuming a straight line following the great circle path
    #source https://www.movable-type.co.uk/scripts/latlong.html
    def initialBearingToGPSPoint(self, point):
        
        #Handle TypeError exceptions when args are not instantiated correctly
        if not isinstance(point, GPSPoint):
            raise TypeError('initialBearingToGPSPoint(self,point) must be passed a GPSPoint class object')
            
        phi1        = radians(self.latitude)
        phi2        = radians(point.latitude)
        d_lambda    = radians(point.longitude - self.longitude)
        
        y = sin(d_lambda) * cos(phi2)
        x = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(d_lambda)
        
        brng = degrees(atan2(y, x)) 
        
        #normalize bearing
        return (brng+360.0) % 360.0
    
    #destinationPointGivenDistanceAndBearingFromStartPoint
    #Function to calculate the destination point (end_point) based on the starting point, distance and bearing
    #source https://www.movable-type.co.uk/scripts/latlong.html
    def destinationPointGivenDistanceAndBearingFromStartPoint(self, distance, bearing):
    
        #Handle TypeError & ValueError exceptions when args are not instantiated correctly
        if not isinstance(float(distance), float) or distance < 0.0:
            raise ValueError('distance must be a number and cannot be less than 0')
        if not isinstance(float(bearing), float) or bearing < 0.0 or bearing > 360.0:
            raise ValueError('bearing must be a number and has to be between 0.0 and 360.0')
        
        ang_d   = distance/earthRadius #angular distance    
        brng    = radians(bearing)
        phi1    = radians(self.latitude)
        lambda1 = radians(self.longitude)
        
        #end point coordinates in radians
        phi2    = asin( sin(phi1)*cos(ang_d) + cos(phi1)*sin(ang_d)*cos(brng) )
        lambda2 = lambda1 + atan2( sin(brng)*sin(ang_d)*cos(phi1), cos(ang_d)-sin(phi1)*sin(phi2) )
        
        #calculate destination point
        destination = GPSPoint( degrees(phi2), degrees(lambda2) )
       
        return destination 
    
    #intermediateGPSPoint
    #returns an intermediate point between a start point (self) and a destination (point) in a fraction of the great circle path denoted by f where 0.0 is the start point and 1.0 is the destination
    #source https://www.movable-type.co.uk/scripts/latlong.html  
    def intermediateGPSPoint(self, point, f):
        
        #Handle TypeError & ValueError exceptions when args are not instantiated correctly
        if not isinstance(point, GPSPoint):
            raise TypeError('intermediateGPSPoint(self, point, f) must be passed a GPSPoint class object')
        if not isinstance(float(f), float) or f < 0.0 or f > 1.0:
            raise ValueError('f (fraction) must be a number and has to be between 0.0 and 1.0')
        
        #get distance
        distance = self.distanceToGPSPoint(point)
         
        ang_d   = distance/earthRadius #angular distance
        phi1    = radians(self.latitude)
        phi2    = radians(point.latitude)
        lambda1 = radians(self.longitude)
        lambda2 = radians(point.longitude)
        
          
        a   = sin((1.0-f)*ang_d)/sin(ang_d)
        b   = sin(f*ang_d)/sin(ang_d)
        x   = a * cos(phi1) * cos(lambda1) + b * cos(phi2) * cos(lambda2)
        y   = a * cos(phi1) * sin(lambda1) + b * cos(phi2) * sin(lambda2)
        z   = a * sin(phi1) + b * sin(phi2)
        
        #calculate coordinates of the intermediate point in radians
        phi_i       = atan2( z, sqrt(x*x+y*y) )
        lambda_i    = atan2( y, x ) 
        
        #calculate intermediate point
        intermediate = GPSPoint( degrees(phi_i), degrees(lambda_i) )
        
        return intermediate
        

#---------------------GPSVector---------------------------------------------------------------------------------------------------------------------------

    
class GPSVector:
    #constructor (2 arguments) that recieves 2 objects with type GPSPoint and calculates magnitude and orientation of GPSVector 
    #constructor (3 arguments) that recieves the starting point (init_point), the magnitude (distance) and orientation (bearing) of a vector and calculates the destination point (end_point)
    def __init__(self, *args, **kwargs):
        
        if len(args) == 2:
            init_point  = args[0]
            end_point   = args[1]
            #Handle TypeError exceptions when args are not instantiated correctly
            if not isinstance(init_point, GPSPoint):
                raise TypeError('init_point must be passed a GPSPoint class object')
            if not isinstance(end_point, GPSPoint):
                raise TypeError('end_point must be passed a GPSPoint class object')
                
            self.init_point = init_point
            self.end_point  = end_point
            
            #calculate vector's magnitude based on the distance between 2 coordinates in meters using the function distanceToGPSPoint() in the GPSPoint class
            self.magnitude  = init_point.distanceToGPSPoint(end_point)
            #calculate vector's orientation based on the initial bearing in the path between 2 coordinates in degrees using the function initialBearingToGPSPoint() in the GPSPoint class
            self.orientation = init_point.initialBearingToGPSPoint(end_point)  
        elif len(args) == 3:
        
            init_point  = args[0]
            magnitude   = args[1]
            orientation = args[2]
        
            #Handle TypeError & ValueError exceptions when args are not instantiated correctly
            if not isinstance(init_point, GPSPoint):
                raise TypeError('init_point must be passed a GPSPoint class object')
            if not isinstance(float(magnitude), float) or magnitude < 0.0:
                raise ValueError('magnitude must be a number and cannot be less than 0')
            if not isinstance(float(orientation), float) or orientation < 0.0 or orientation > 360.0:
                raise ValueError('orientation must be a number and has to be between 0.0 and 360.0')
            
            self.init_point  = init_point
            self.magnitude   = magnitude
            self.orientation = orientation
            
            #calculate end_point using destinationPointGivenDistanceAndBearingFromStartPoint()
            self.end_point   = init_point.destinationPointGivenDistanceAndBearingFromStartPoint(self.magnitude, self.orientation)
        else:
            raise TypeError('GPSVector constructor: Number of arguments is invalid')
    
    def __str__(self):
        
        msg = "iP = %s, eP = %s, mag = %.2f, ori = %.2f" % (self.init_point, self.end_point, self.magnitude, self.orientation)
        return msg
        
    #getListOfIntermediatePoints()
    #returns a list of intermediate points (length of the list specified by numberOfIntermediatePoints) taking the init_point and end_point of self as arguments using the function intermediateGPSPoint(self, point, f) from the GPSPoint class 
    #the list contains numberOfIntermediatePoints intermediate points that are equidistant between them
    def getListOfIntermediatePoints(self, numberOfIntermediatePoints):
        
        #Handle TypeError & ValueError exceptions when args are not instantiated correctly
        if not isinstance(numberOfIntermediatePoints, int) or numberOfIntermediatePoints <= 0:
            raise ValueError('numberOfIntermediatePoints must be an int and has to be greater than 0')
        
        #list of points
        points = []
        
        #first element
        points.append(self.init_point)
        
        #calculate fraction delta
        delta = 1.0/(numberOfIntermediatePoints+1.0)
        
        #iterate over each point invoking the function intermediateGPSPoint() for each value of delta referred as f
        for i in range(1, numberOfIntermediatePoints+1):
            point = self.init_point.intermediateGPSPoint(self.end_point, delta*float(i))
            points.append(point)
        
        #last element 
        points.append(self.end_point)
        
        return points
        
    #getListOfIntermediatePointsBasedOnDistance()
    #returns a list of intermediate points taking the init_point and end_point of self as arguments using the function intermediateGPSPoint(self, point, f) from the GPSPoint class 
    #the list is generated taking in account that the intermediate points must be spaced by distanceBetweenPoints between them
    #this function uses getListOfIntermediatePoints() once it has determined how many intermediate points are needed to match the criteria of spacing by distance
    def getListOfIntermediatePointsBasedOnDistance(self, distanceBetweenPoints):
        
        #Handle TypeError & ValueError exceptions when args are not instantiated correctly
        if not isinstance(float(distanceBetweenPoints), float) or distanceBetweenPoints <= 0:
            raise ValueError('distanceBetweenPoints must be a number and has to be greater than 0.0')
        
        #calculate numberOfIntermediatePoints
        numberOfIntermediatePoints = int(round(self.magnitude/distanceBetweenPoints))-1

        return self.getListOfIntermediatePoints(numberOfIntermediatePoints)  
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
