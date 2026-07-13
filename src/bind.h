#ifndef BIND_H
#define BIND_H

#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>

// Module types
enum ModuleType {
    MODULE_BODY_CONTROLLER,
    MODULE_SENSOR_MODULE,
    MODULE_MESH,
    MODULE_REMOTE_FOB
};

struct BindInfo {
    bool bound;
    String vin;
    String moduleId;
    String mac;
};

class ModuleBinder {
public:
    ModuleBinder() {}

    void begin(ModuleType type) {
        m_type = type;
        m_info.bound = false;
        m_info.mac = WiFi.macAddress();
        m_info.mac.replace(":", "");
        m_info.mac.toUpperCase();

        // Load from NVS
        m_prefs.begin("carputer", true);  // read-only
        m_info.bound = m_prefs.getBool("bound", false);
        m_info.vin = m_prefs.getString("vin", "");
        m_info.moduleId = m_prefs.getString("moduleId", "");
        m_prefs.end();

        Serial.printf("[BIND] Loaded: bound=%d vin=%s id=%s mac=%s\n",
                      m_info.bound, m_info.vin.c_str(),
                      m_info.moduleId.c_str(), m_info.mac.c_str());
    }

    // Generate moduleId from prefix + MAC + VIN + timestamp
    String generateId(const String& vinStr) {
        const char* prefixes[] = {"BC", "SM", "MK", "RK"};
        String prefix = prefixes[m_type];

        // Simple hash from MAC + VIN + millis
        String input = m_info.mac + vinStr + String(millis());
        unsigned long hash = 5381;
        for (unsigned int i = 0; i < input.length(); i++) {
            hash = ((hash << 5) + hash) + input[i];
        }
        char buf[16];
        snprintf(buf, sizeof(buf), "%s-%08X", prefix.c_str(), hash);
        return String(buf);
    }

    // Bind to VIN — stores in NVS, returns true on success
    bool bind(const String& vinStr) {
        m_info.vin = vinStr;
        m_info.moduleId = generateId(vinStr);
        m_info.bound = true;

        m_prefs.begin("carputer", false);
        m_prefs.putBool("bound", true);
        m_prefs.putString("vin", m_info.vin);
        m_prefs.putString("moduleId", m_info.moduleId);
        m_prefs.end();

        Serial.printf("[BIND] Bound: id=%s vin=%s\n",
                      m_info.moduleId.c_str(), m_info.vin.c_str());
        return true;
    }

    // Unbind — clears NVS
    void unbind() {
        m_prefs.begin("carputer", false);
        m_prefs.putBool("bound", false);
        m_prefs.putString("vin", "");
        m_prefs.putString("moduleId", "");
        m_prefs.end();

        m_info.bound = false;
        m_info.vin = "";
        m_info.moduleId = "";

        Serial.println("[BIND] Unbound");
    }

    // Check if VIN matches (for module swap detection)
    bool vinMatches(const String& vinStr) const {
        if (!m_info.bound) return true;  // unbound modules accept any VIN
        return m_info.vin == vinStr;
    }

    bool isBound() const { return m_info.bound; }
    String vin() const { return m_info.vin; }
    String moduleId() const { return m_info.moduleId; }
    String mac() const { return m_info.mac; }
    const BindInfo& info() const { return m_info; }

    // Build JSON bind response
    void sendBoundResponse(WiFiClient& tcpClient) {
        if (!tcpClient.connected()) return;
        String resp = "{\"event\":\"bound\",\"moduleId\":\"" + m_info.moduleId +
                      "\",\"mac\":\"" + m_info.mac + "\",\"vin\":\"" + m_info.vin + "\"}";
        tcpClient.println(resp);
        Serial.println("[BIND] Sent bound response to TCP client");
    }

    void sendBoundResponse(WiFiUDP& udp, const IPAddress& ip, int port) {
        String resp = "{\"event\":\"bound\",\"moduleId\":\"" + m_info.moduleId +
                      "\",\"mac\":\"" + m_info.mac + "\",\"vin\":\"" + m_info.vin + "\"}";
        udp.beginPacket(ip, port);
        udp.print(resp);
        udp.println();
        udp.endPacket();
        Serial.println("[BIND] Sent bound response via UDP");
    }

    void sendUnboundResponse(WiFiClient& tcpClient) {
        if (!tcpClient.connected()) return;
        tcpClient.println("{\"event\":\"unbound\"}");
    }

    void sendUnboundResponse(WiFiUDP& udp, const IPAddress& ip, int port) {
        udp.beginPacket(ip, port);
        udp.print("{\"event\":\"unbound\"}");
        udp.println();
        udp.endPacket();
    }

    void sendErrorResponse(WiFiClient& tcpClient, const String& error) {
        if (!tcpClient.connected()) return;
        String resp = "{\"event\":\"bind_error\",\"error\":\"" + error + "\"}";
        tcpClient.println(resp);
    }

    void sendErrorResponse(WiFiUDP& udp, const IPAddress& ip, int port, const String& error) {
        String resp = "{\"event\":\"bind_error\",\"error\":\"" + error + "\"}";
        udp.beginPacket(ip, port);
        udp.print(resp);
        udp.println();
        udp.endPacket();
    }

private:
    Preferences m_prefs;
    ModuleType m_type;
    BindInfo m_info;
};

#endif // BIND_H
