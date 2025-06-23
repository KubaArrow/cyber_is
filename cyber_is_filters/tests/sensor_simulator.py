#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt16MultiArray, Float64MultiArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class LineDetectorSimulator:
    def __init__(self):
        rospy.init_node('line_detector_simulator')
        # subskrypcja odometrii
        self.sub_odom   = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        # publisher detektora linii
        self.pub_line   = rospy.Publisher('/line_detector', UInt16MultiArray, queue_size=10)
        # publisher sygnału magnetycznego
        self.pub_magnet = rospy.Publisher('/magnet', Float64MultiArray, queue_size=10)
        # publisher markerów do RViz
        self.pub_marker = rospy.Publisher('/line_marker', Marker, queue_size=10)

        # wymiary dużego prostokąta (linia)
        self.w  = 3.03 / 2.0   # połowa szerokości
        self.h  = 6.06         # wysokość
        # wymiary małego prostokąta (taśma magnetyczna)
        self.w2 = 0.10         # szerokość
        self.h2 = 0.60         # wysokość
        # próg detekcji [m]
        self.thresh = 0.015

        # przygotuj obydwa Markery
        self._create_line_marker()
        self._create_magnet_marker()

        # co 1 s publikuj markery do RViz
        rospy.Timer(rospy.Duration(1.0), self._timer_publish_markers)

        rospy.loginfo("Line + Magnet Simulator started")
        rospy.spin()

    def _create_line_marker(self):
        m = Marker()
        m.header.frame_id = 'odom'
        m.ns  = 'line'
        m.id  = 0
        m.type   = Marker.LINE_STRIP
        m.action = Marker.ADD

        # prostokąt przed obrotem
        pts = [
            Point(-self.w, 0.0, 0.0),
            Point( self.w, 0.0, 0.0),
            Point( self.w,   self.h, 0.0),
            Point(-self.w,   self.h, 0.0),
            Point(-self.w, 0.0, 0.0),
        ]
        m.points = pts

        # obrót −90° wokół Z
        m.pose.orientation.x = 0.0
        m.pose.orientation.y = 0.0
        m.pose.orientation.z = -0.7071068
        m.pose.orientation.w =  0.7071068

        m.scale.x = 0.05
        m.color.r = 1.0
        m.color.a = 1.0

        self.marker_line = m

    def _create_magnet_marker(self):
        m = Marker()
        m.header.frame_id = 'odom'
        m.ns  = 'magnet'
        m.id  = 1
        m.type   = Marker.LINE_STRIP
        m.action = Marker.ADD

        # lokalne współrzędne małego prostokąta w prawym górnym narożniku wewnątrz dużego:
        # dolny-lewy:
        x0 =  self.w - self.w2
        y0 =  self.h - self.h2
        pts = [
            Point(x0,           y0,           0.0),
            Point(x0 + self.w2, y0,           0.0),
            Point(x0 + self.w2, y0 + self.h2, 0.0),
            Point(x0,           y0 + self.h2, 0.0),
            Point(x0,           y0,           0.0),
        ]
        m.points = pts

        # ten sam obrót co dla linii
        m.pose.orientation.x = 0.0
        m.pose.orientation.y = 0.0
        m.pose.orientation.z = -0.7071068
        m.pose.orientation.w =  0.7071068

        m.scale.x = 0.03   # cieńsza kreska
        m.color.g = 1.0    # zielony
        m.color.a = 1.0

        self.marker_magnet = m

    def _timer_publish_markers(self, _):
        now = rospy.Time.now()
        # duży prostokąt
        self.marker_line.header.stamp   = now
        self.pub_marker.publish(self.marker_line)
        # mały prostokąt
        self.marker_magnet.header.stamp = now
        self.pub_marker.publish(self.marker_magnet)

    def odom_callback(self, msg):
        # globalne współrzędne robota
        xg = msg.pose.pose.position.x
        yg = msg.pose.pose.position.y
        # lokalne współrzędne względem marker_frame (obrót +90° do detekcji)
        lx = -yg
        ly =  xg

        # — DETEKCJA „LINII” (krawędzie dużego prostokąta) — 
        on_line = (
            (abs(ly -   0.0) <= self.thresh and abs(lx)          <= self.w) or
            (abs(ly - self.h)  <= self.thresh and abs(lx)          <= self.w) or
            (abs(lx + self.w)  <= self.thresh and 0.0 <= ly <= self.h) or
            (abs(lx - self.w)  <= self.thresh and 0.0 <= ly <= self.h)
        )
        if on_line:
            arr = UInt16MultiArray()
            arr.data = [4000] * 5
            self.pub_line.publish(arr)

        # — DETEKCJA „TAŚMY MAGNETYCZNEJ” (obszar małego prostokąta) — 
        if (self.w - self.w2) <= lx <= self.w and (self.h - self.h2) <= ly <= self.h:
            mag = Float64MultiArray()
            mag.data = [10000.0] * 3
            self.pub_magnet.publish(mag)

if __name__ == '__main__':
    try:
        LineDetectorSimulator()
    except rospy.ROSInterruptException:
        pass
