#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import struct
import sensor_msgs_py.point_cloud2 as pc2  # Utilidad para crear nubes de puntos

class FusionPointCloudNode(Node):
    def __init__(self):
        super().__init__('fuse_image')

        # Suscriptores para la imagen RGB y la imagen de profundidad
        self.rgb_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',  # Ajusta el tópico RGB según tu configuración
            self.rgb_callback,
            10
        )
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',  # Tópico de profundidad
            self.depth_callback,
            10
        )

        # Publicador de la nube de puntos
        self.pc_pub = self.create_publisher(PointCloud2, '/point_cloud', 10)

        self.bridge = CvBridge()
        self.rgb_image = None
        self.depth_image = None

        # Parámetros de cámara (intrínsecos); ajústalos según tu cámara
        self.fx = 616.0
        self.fy = 616.0
        self.cx = 320.0
        self.cy = 240.0

    def rgb_callback(self, msg):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"Error al convertir la imagen RGB: {e}")
        self.try_publish_pointcloud()

    def depth_callback(self, msg):
        try:
            # Se utiliza 'passthrough' para obtener los valores originales
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            self.get_logger().error(f"Error al convertir la imagen de profundidad: {e}")
        self.try_publish_pointcloud()

    def try_publish_pointcloud(self):
        # Se necesita disponer de ambas imágenes para procesar la nube de puntos
        if self.rgb_image is None or self.depth_image is None:
            return

        depth_h, depth_w = self.depth_image.shape
        rgb_h, rgb_w, _ = self.rgb_image.shape

        # Si los tamaños de las imágenes no coinciden, se redimensiona la RGB a la resolución de la profundidad
        if (depth_h != rgb_h) or (depth_w != rgb_w):
            self.get_logger().warn("Las resoluciones de RGB y profundidad difieren. Redimensionando RGB.")
            self.rgb_image = cv2.resize(self.rgb_image, (depth_w, depth_h))

        points = []
        step = 2  # Para reducir la cantidad de puntos (procesa cada 2do píxel)

        # Recorremos la imagen (puedes optimizar este proceso si es necesario)
        for v in range(0, depth_h, step):
            for u in range(0, depth_w, step):
                depth = self.depth_image[v, u]
                # Si el valor de profundidad es 0, se ignora (sin dato válido)
                if depth == 0:
                    continue
                # Convertir la profundidad de milímetros a metros
                z = depth * 0.001
                x = (u - self.cx) * z / self.fx
                y = (v - self.cy) * z / self.fy

                # Obtener el color del píxel correspondiente en la imagen RGB
                b, g, r = self.rgb_image[v, u]
                # Empaquetar el color en un entero (estilo ROS: cada canal en un byte y alfa en 255)
                rgb_packed = struct.unpack('I', struct.pack('BBBB', int(r), int(g), int(b), 255))[0]

                points.append([x, y, z, rgb_packed])

        # Crear el encabezado del mensaje
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "camera_link"  # Ajusta el frame según tu TF

        # Definir los campos de la nube de puntos (x, y, z y rgb)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)
        ]

        # Crear el mensaje PointCloud2 utilizando la utilidad pc2
        pc2_msg = pc2.create_cloud(header, fields, points)
        self.pc_pub.publish(pc2_msg)
        self.get_logger().info(f"Nube de puntos publicada con {len(points)} puntos.")

        # Reiniciar las imágenes para procesar el siguiente par
        self.rgb_image = None
        self.depth_image = None

def main(args=None):
    rclpy.init(args=args)
    node = FusionPointCloudNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
