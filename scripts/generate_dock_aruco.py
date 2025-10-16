#!/usr/bin/env python3

import cv2
import numpy as np
from cv2 import aruco
import os

def generate_dock_aruco():
    """Gera marcador ArUco específico para a charging dock"""
    
    # Configuração do ArUco
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    marker_id = 15  # ID específico para a charging dock
    marker_size = 800  # Tamanho em pixels
    
    # Gerar marcador (versão compatível)
    marker_image = cv2.aruco.drawMarker(aruco_dict, marker_id, marker_size)
    
    # Adicionar borda branca para melhor detecção
    border_size = 80
    bordered_image = np.ones((marker_size + 2*border_size, 
                             marker_size + 2*border_size), dtype=np.uint8) * 255
    bordered_image[border_size:border_size+marker_size, 
                   border_size:border_size+marker_size] = marker_image
    
    # Caminho de saída
    output_dir = "../models/charging_dock/materials/textures"
    os.makedirs(output_dir, exist_ok=True)
    output_path = f"{output_dir}/aruco_dock_marker.png"
    
    # Salvar
    cv2.imwrite(output_path, bordered_image)
    
    print(f"✅ ArUco marker {marker_id} para charging dock criado!")
    print(f"   📁 Salvo em: {output_path}")
    print(f"   📐 Tamanho: {marker_size}x{marker_size} pixels")
    print(f"   🎯 Use ID {marker_id} para detectar a charging dock")
    print(f"   📋 Comando de teste: ros2 run sucata dock_detector.py")

if __name__ == "__main__":
    generate_dock_aruco()