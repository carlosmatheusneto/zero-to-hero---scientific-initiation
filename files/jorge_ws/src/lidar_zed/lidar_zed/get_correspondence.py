#!/usr/bin/env python3
import os
import cv2
import numpy as np

from lidar_zed.read_yaml import extract_configuration
config = extract_configuration()

class CorrespondenceBuilder:
    def __init__(self, data_dir, chessboard_rows, chessboard_cols, square_size):
        self.data_dir = config['general']['data_folder']
        self.corr_file = config['general']['correspondence_file']
        self.chessboard_rows = ['chessboard']['rows']
        self.chessboard_cols = ['chessboard']['columns']
        self.square_size = ['chessboard']['square_size_m']

        if not os.path.exists(self.data_dir):
            os.makedirs(self.data_dir)

        # Cria ou reseta o arquivo de correspondência
        with open(self.corr_file, "w") as f:
            f.write("# u,v,X,Y,Z\n")

    def process_pair(self, image_path, lidar_path):
        # --- Carrega imagem ---
        img = cv2.imread(image_path)
        if img is None:
            print(f"Erro ao carregar imagem {image_path}")
            return False

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (self.chessboard_cols, self.chessboard_rows), None)

        if not ret:
            print(f"Nenhum tabuleiro detectado em {image_path}")
            return False

        corners = cv2.cornerSubPix(
            gray, corners, (11,11), (-1,-1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        )

        # --- Carrega LiDAR ---
        lidar_data = np.load(lidar_path)
        ranges = lidar_data["ranges"]
        angles = lidar_data["angles"]

        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)
        zs = np.zeros_like(xs)
        lidar_points = np.vstack((xs, ys, zs)).T  # apenas referência, não usado no arquivo

        # --- Cria pontos 3D do tabuleiro ---
        objp = np.zeros((self.chessboard_rows * self.chessboard_cols, 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.chessboard_cols, 0:self.chessboard_rows].T.reshape(-1, 2)
        objp *= self.square_size

        # --- Salva correspondências ---
        with open(self.corr_file, "a") as f:
            for i in range(len(corners)):
                u, v = corners[i,0]
                X, Y, Z = objp[i]
                f.write(f"{u},{v},{X},{Y},{Z}\n")

        print(f"Correspondences adicionadas de {image_path} e {lidar_path}")
        return True

    def build(self):
        files = os.listdir(self.data_dir)
        images = sorted([f for f in files if f.endswith(".png")])
        lidars = sorted([f for f in files if f.endswith("_lidar.npz")])

        for img, lidar in zip(images, lidars):
            self.process_pair(os.path.join(self.data_dir, img), os.path.join(self.data_dir, lidar))

        print(f"\n✅ Correspondence file salvo em: {self.corr_file}")


if __name__ == "__main__":
    DATA_DIR = config['general']['data_folder']
    builder = CorrespondenceBuilder(DATA_DIR)
    builder.build()
