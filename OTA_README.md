# OTA (Over-The-Air) Update Guide

## 📡 Fonctionnalité OTA Implémentée

Ce projet inclut maintenant une fonctionnalité complète de mise à jour OTA (Over-The-Air) qui permet de mettre à jour le firmware de ton ESP32 LoRa sans connection physique.

### Caractéristiques
- ✅ Vérification automatique des mises à jour (toutes les heures)
- ✅ Téléchargement HTTP/HTTPS avec streaming (pas de problème mémoire)
- ✅ Support de l'authentification HTTP Basic
- ✅ Support SSL avec vérification de certificat (optionnel)
- ✅ Affichage OLED des étapes de mise à jour
- ✅ Redémarrage automatique après mise à jour réussie
- ✅ Gestion complète des erreurs

---

## 🚀 Configuration Rapide

### 1. Configurer les credentials OTA

Édite le fichier `include/credentials/credentials.h` :

```cpp
// OTA Update Configuration
#define OTA_ENABLED      true
#define OTA_SERVER       "192.168.1.100"  // IP de ton PC/serveur
#define OTA_PORT         8000
#define OTA_PATH         "/firmware.bin"
//#define OTA_FINGERPRINT  "A1:B2:C3:D4:E5:F6:78:90:12:34:56:78:90:AB:CD:EF:12:34:56:78"  // SHA-1 fingerprint du certificat SSL
//#define OTA_USERNAME     "ota_user"  // Auth HTTP Basic (optionnel)
//#define OTA_PASSWORD     "ota_pass"  // Auth HTTP Basic (optionnel)
#define OTA_CHECK_INTERVAL 30000  // Vérification toutes les 30 secondes (pour les tests)
//#define OTA_SKIP_CERT_CHECK  // Décommenter pour ignorer la vérification SSL (NON RECOMMANDÉ)
```

> **Astuce pour les tests** : Réduis `OTA_CHECK_INTERVAL` à 30000 (30 secondes) pour tester rapidement.

---

## 🌐 Démarrer le Serveur OTA

### Méthode 1 : Utiliser le script Python fourni

1. **Builder le firmware** (si ce n'est pas déjà fait) :
   ```bash
   pio run
   ```

2. **Démarrer le serveur OTA** :
   ```bash
   # Sur Windows (double-clique)
   run_ota_server.bat
   
   # Ou en ligne de commande
   python ota_server.py --port 8000
   ```

3. **Trouver l'adresse IP de ton PC** :
   - Windows : `ipconfig` (cherche "Adresse IPv4")
   - Linux/macOS : `ifconfig` ou `ip a`

4. **Mettre à jour OTA_SERVER dans credentials.h** avec cette IP

5. **Rebuild et upload le firmware initial** :
   ```bash
   pio run --target upload
   ```

### Méthode 2 : Utiliser un serveur web existant (Apache/Nginx)

1. Copier `firmware.bin` dans le dossier web (ex: `/var/www/html/`)
2. Créer un endpoint API (PHP, Node.js, etc.) qui retourne :
   ```json
   {
     "available": true,
     "version": "1.1.0",
     "url": "/firmware.bin"
   }
   ```
3. Configurer `OTA_SERVER`, `OTA_PORT`, `OTA_PATH` en conséquence

---

## 📋 API du Serveur OTA

Le serveur OTA fourni implémente ces endpoints :

| Endpoint | Méthode | Description |
|----------|---------|-------------|
| `/api/firmware/check?device=ESP32_LoRa_broker&version=X.X.X` | GET | Vérifie si une mise à jour est disponible |
| `/api/firmware/version?device=ESP32_LoRa_broker&version=X.X.X` | GET | Alternative pour vérifier la version |
| `/firmware.bin` | GET | Télécharge le firmware |

### Réponse de l'API

```json
{
  "available": true,
  "version": "1.1.0",
  "device": "ESP32_LoRa_broker",
  "url": "/firmware.bin"
}
```

---

## 🔧 Personnalisation du Serveur OTA

Tu peux modifier le comportement du serveur Python :

```bash
# Démarrer sur un port différent
python ota_server.py --port 8080

# Spécifier un chemin différent pour le firmware
python ota_server.py --firmware_path my_firmware/firmware.bin

# Changer la version du firmware (pour forcer une mise à jour)
python ota_server.py --version 1.2.0
```

---

## 🔍 Dépannage

### Problème : L'ESP32 ne vérifie pas les mises à jour

**Solutions** :
1. Vérifie que `OTA_ENABLED` est à `true` dans credentials.h
2. Vérifie que `WiFiConnect()` fonctionne (l'ESP32 doit être connecté au WiFi)
3. Réduis `OTA_CHECK_INTERVAL_MS` pour les tests (ex: 30000 pour 30 secondes)
4. Vérifie dans le moniteur série les logs OTA

### Problème : La vérification échoue avec une erreur HTTP

**Solutions** :
1. Vérifie que le serveur OTA est en cours d'exécution
2. Vérifie que l'IP dans `OTA_SERVER` est correcte
3. Vérifie que le port dans `OTA_PORT` correspond au serveur
4. Si tu utilises HTTPS, vérifie que `OTA_FINGERPRINT` est configuré ou que `OTA_SKIP_CERT_CHECK` est décommenté

### Problème : Le téléchargement du firmware échoue

**Solutions** :
1. Vérifie que le chemin dans `OTA_PATH` est correct
2. Vérifie que le fichier `firmware.bin` existe sur le serveur
3. Vérifie que le serveur supporte les requêtes GET pour le fichier
4. Vérifie que l'ESP32 a assez de mémoire (Flash: 75% utilisé, il reste ~325KB)

### Problème : L'OTA fonctionne mais l'ESP32 ne redémarre pas

**Solutions** :
1. Vérifie que `Update.end(true)` retourne true (le paramètre `true` déclenche le redémarrage)
2. Vérifie que le firmware téléchargé est valide
3. Ajoute des logs dans `performOTAUpdate()` pour voir où ça bloque

---

## 📊 Logs et Débogage

Active le mode DEBUG dans le code (déjà activé) et observe le moniteur série :

```
--> Starting OTA update check
OTA Check URL: http://192.168.1.100:8000/api/firmware/check?device=ESP32_LoRa_broker&version=1.0.0
OTA Version Check Response: {"available": true, "version": "1.1.0", ...}
Update available, starting download...
Firmware URL: http://192.168.1.100:8000/firmware.bin
OTA: Firmware size: 992304 bytes
OTA: Writing firmware...
OTA: Update completed successfully!
Rebooting after OTA update...
```

Sur l'écran OLED, tu verras :
- "OTA" / "Check..." → Vérification en cours
- "OTA" / "Download..." → Téléchargement
- "OTA" / "Update" / "OK!" → Mise à jour réussie
- "Reboot" / "in 5s" → Redémarrage
- "OTA" / "Up2date" → Déjà à jour
- "OTA" / "Error" → Erreur

---

## 🛡️ Sécurité en Production

Pour une utilisation en production, il est **fortement recommandé** de :

1. **Utiliser HTTPS** au lieu de HTTP
2. **Configurer OTA_FINGERPRINT** avec l'empreinte SHA-1 de ton certificat SSL
3. **Activer l'authentification** avec `OTA_USERNAME` et `OTA_PASSWORD`
4. **Ne pas utiliser OTA_SKIP_CERT_CHECK** (décommenter seulement pour les tests)
5. **Signer le firmware** (non implémenté ici, mais recommandé pour la sécurité)

### Générer une empreinte SSL

Pour obtenir l'empreinte SHA-1 de ton certificat SSL :

```bash
openssl s_client -connect ton-serveur.com:443 -showcerts < /dev/null 2>/dev/null | openssl x509 -fingerprint -sha1 -noout
```

Puis copie la chaîne entre les `:` dans `OTA_FINGERPRINT` (sans les `:`)

Exemple :
```cpp
#define OTA_FINGERPRINT "A1B2C3D4E5F678901234567890ABCDEF12345678"
```

---

## 📈 Optimisation

### Réduire la taille du firmware

Le firmware actuel utilise 75% du flash. Pour libérer de l'espace :

1. **Désactiver le DEBUG** dans main.cpp pour la production
2. **Utiliser un niveau d'optimisation plus élevé** dans platformio.ini
3. **Supprimer les bibliothèques inutilisées**

### Augmenter la vitesse de téléchargement

Le téléchargement utilise le WiFi. Pour améliorer la vitesse :
1. Assure-toi d'avoir un bon signal WiFi
2. Utilise un serveur sur le même réseau local
3. Évite les réseaux WiFi saturés

---

## 🎯 Workflow de Déploiement

### Développement
1. Modifie le code
2. Builder : `pio run`
3. Upload manuellement : `pio run --target upload`
4. Teste le nouveau firmware

### Production (avec OTA)
1. Modifie le code
2. Incrémente la version dans `ota_server.py` (ex: `--version 1.1.0`)
3. Builder : `pio run` (génère nouveau firmware.bin)
4. Redémarre le serveur OTA (si tu utilises le script Python)
5. Attends que l'ESP32 vérifie et télécharge automatiquement la mise à jour
6. L'ESP32 redémarre avec le nouveau firmware

---

## 📚 Références

- [ESP32 OTA Update Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/ota.html)
- [PlatformIO OTA Example](https://docs.platformio.org/en/latest/examples/ota-update.html)
- [Arduino ESP32 Update Library](https://github.com/espressif/arduino-esp32/tree/master/libraries/Update)

---

**Bonnes mises à jour OTA !** 🚀
