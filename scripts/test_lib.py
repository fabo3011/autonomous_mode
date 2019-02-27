#!/usr/bin/env python

from lib.GPSVector import *



if __name__ == '__main__':
    
    p = GPSPoint(20.614219,-100.404987)    
    p2 = GPSPoint(20.611005,-100.406115)
    
    Vec = GPSVector(p,p2)
    
    print(Vec)
    
    Vec2 = GPSVector(p, 376, 198)
    print(Vec2)
    
    
    points = Vec.getListOfIntermediatePointsBasedOnDistance(5)

    lista = GPSListOfPoints(points)

    print(lista)

    lista.writeListToFile('test.txt')

    lista.loadListFromFile('log.txt')
    print(lista)
    
    '''
    print(len(points))

    for i in range(0, len(points)):
        print(points[i])
    '''
    
