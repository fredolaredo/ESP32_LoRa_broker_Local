#!/usr/bin/env python3
"""
Serveur OTA pour ESP32 LoRa Broker

Ce serveur permet de tester la fonctionnalité OTA en servant :
- Le firmware.bin à télécharger
- Une API pour vérifier si une mise à jour est disponible

Utilisation :
    python ota_server.py [--port 8000] [--firmware_path .pio/build/ttgo-lora32-v1/firmware.bin]

Puis configurez credentials.h avec :
    #define OTA_SERVER "192.168.x.x"  (adresse IP de ce serveur)
    #define OTA_PORT 8000
    #define OTA_PATH "/firmware.bin"
"""

import argparse
import json
import os
import threading
from http.server import HTTPServer, SimpleHTTPRequestHandler
from socketserver import ThreadingMixIn
import time

# Configuration
DEFAULT_PORT = 8000
DEFAULT_FIRMWARE_PATH = ".pio/build/ttgo-lora32-v1/firmware.bin"

# Version du firmware actuelle (à incrementer pour forcer une mise à jour)
CURRENT_FIRMWARE_VERSION = "1.1.0"
DEVICE_NAME = "ESP32_LoRa_broker"

class OTARequestHandler(SimpleHTTPRequestHandler):
    """Gestionnaire de requêtes HTTP pour le serveur OTA"""
    
    # Désactiver les logs par défaut
    def log_message(self, format, *args):
        print(f"[OTA Server] {self.client_address[0]} - {format % args}")
    
    def do_GET(self):
        """Traite les requêtes GET"""
        print(f"\n[OTA] GET {self.path}")
        
        # API pour vérifier si une mise à jour est disponible
        if self.path.startswith('/api/firmware/check'):
            self.handle_firmware_check()
        # API alternative pour vérifier la version
        elif self.path.startswith('/api/firmware/version'):
            self.handle_firmware_check()
        # Téléchargement du firmware
        elif self.path == '/' + os.path.basename(firmware_path) or self.path == firmware_path:
            self.serve_firmware()
        # Autre fichier statique
        else:
            super().do_GET()
    
    def handle_firmware_check(self):
        """Vérifie si une mise à jour est disponible"""
        # Analyser les paramètres de la requête
        from urllib.parse import urlparse, parse_qs
        parsed = urlparse(self.path)
        params = parse_qs(parsed.query)
        
        # Extraire la version actuelle du device
        current_version = params.get('version', ['0.0.0'])[0]
        device = params.get('device', [''])[0]
        
        print(f"[OTA] Device: {device}, Current version: {current_version}")
        
        # Comparer les versions (simplifié)
        # En production, utilisez une comparaison sémantique propre
        update_available = (current_version != CURRENT_FIRMWARE_VERSION)
        
        if update_available:
            print(f"[OTA] Update available! New version: {CURRENT_FIRMWARE_VERSION}")
        else:
            print(f"[OTA] No update available")
        
        # Construire la réponse JSON
        response = {
            "available": update_available,
            "version": CURRENT_FIRMWARE_VERSION,
            "device": device,
            "url": "/" + os.path.basename(firmware_path)
        }
        
        # Envoyer la réponse
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(json.dumps(response, indent=2).encode('utf-8'))
    
    def serve_firmware(self):
        """Sert le fichier firmware.bin"""
        print(f"[OTA] Serving firmware: {firmware_path}")
        
        if not os.path.exists(firmware_path):
            self.send_error(404, f"Firmware file not found: {firmware_path}")
            return
        
        try:
            # Envoyer le fichier
            self.send_response(200)
            self.send_header('Content-Type', 'application/octet-stream')
            self.send_header('Content-Disposition', f'attachment; filename="{os.path.basename(firmware_path)}"')
            self.end_headers()
            
            with open(firmware_path, 'rb') as f:
                self.copyfile(f, self.wfile)
            
            print(f"[OTA] Firmware sent successfully ({os.path.getsize(firmware_path)} bytes)")
        except Exception as e:
            print(f"[OTA] Error serving firmware: {e}")
            self.send_error(500, f"Error serving firmware: {e}")


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Serveur HTTP multi-thread pour gérer plusieurs requêtes"""
    allow_reuse_address = True
    daemon_threads = True


def run_server(port, firmware_path):
    """Démarre le serveur OTA"""
    global firmware_path
    
    # Vérifier que le firmware existe
    if not os.path.exists(firmware_path):
        print(f"\n[ERROR] Firmware file not found: {firmware_path}")
        print(f"Please build the project first with: pio run")
        print(f"Or specify the correct path with --firmware_path")
        return
    
    firmware_size = os.path.getsize(firmware_path)
    print(f"\n{'='*60}")
    print(f"OTA Server for ESP32 LoRa Broker")
    print(f"{'='*60}")
    print(f"Firmware: {firmware_path}")
    print(f"Firmware size: {firmware_size:,} bytes")
    print(f"Current version: {CURRENT_FIRMWARE_VERSION}")
    print(f"Server port: {port}")
    print(f"{'='*60}")
    print(f"\nAPI Endpoints:")
    print(f"  GET /api/firmware/check?device={DEVICE_NAME}&version=X.X.X")
    print(f"     -> Returns if update is available")
    print(f"  GET /{os.path.basename(firmware_path)}")
    print(f"     -> Downloads the firmware")
    print(f"\nConfigure your ESP32 with:")
    print(f"  #define OTA_SERVER \"<this-server-ip>\"")
    print(f"  #define OTA_PORT {port}")
    print(f"  #define OTA_PATH \"/{os.path.basename(firmware_path)}\"")
    print(f"{'='*60}\n")
    
    # Démarrer le serveur
    server_address = ('', port)
    httpd = ThreadedHTTPServer(server_address, OTARequestHandler)
    
    try:
        print(f"[OTA Server] Starting on port {port}...")
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[OTA Server] Shutting down...")
        httpd.shutdown()
        print("[OTA Server] Stopped")


if __name__ == '__main__':
    # Parser les arguments
    parser = argparse.ArgumentParser(description='OTA Server for ESP32 LoRa Broker')
    parser.add_argument('--port', type=int, default=DEFAULT_PORT, 
                        help=f'Server port (default: {DEFAULT_PORT})')
    parser.add_argument('--firmware_path', type=str, default=DEFAULT_FIRMWARE_PATH,
                        help=f'Path to firmware.bin (default: {DEFAULT_FIRMWARE_PATH})')
    parser.add_argument('--version', type=str, default=CURRENT_FIRMWARE_VERSION,
                        help=f'Current firmware version (default: {CURRENT_FIRMWARE_VERSION})')
    
    args = parser.parse_args()
    
    # Mettre à jour la version
    CURRENT_FIRMWARE_VERSION = args.version
    
    # Démarrer le serveur
    run_server(args.port, args.firmware_path)
