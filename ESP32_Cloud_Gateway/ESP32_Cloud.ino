#include <HardwareSerial.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <ArduinoJson.h>

HardwareSerial mySerial(2);  // Use UART2 (GPIO16=RX, GPIO17=TX)

// WiFi and AWS IoT configuration
const char* ssid = "realme 5i";
const char* password = "s.n.s.03";
const char* awsEndpoint = "a1ggbc06nj1m3p-ats.iot.ap-south-1.amazonaws.com";

// Certificates and keys
static const char certificatePemCrt[] PROGMEM = R"KEY(
-----BEGIN CERTIFICATE-----
MIIDWjCCAkKgAwIBAgIVAPBVdosabr+JH5Mp8REbcqaiLcZGMA0GCSqGSIb3DQEB
CwUAME0xSzBJBgNVBAsMQkFtYXpvbiBXZWIgU2VydmljZXMgTz1BbWF6b24uY29t
IEluYy4gTD1TZWF0dGxlIFNUPVdhc2hpbmd0b24gQz1VUzAeFw0yNTAyMDUwOTM2
MzFaFw00OTEyMzEyMzU5NTlaMB4xHDAaBgNVBAMME0FXUyBJb1QgQ2VydGlmaWNh
dGUwggEiMA0GCSqGSIb3DQEBAQUAA4IBDwAwggEKAoIBAQCu34eGgXJEs7r8AYak
S2eKrncURTDNXi+75ANSBJIS1Khd4dBpdKWfjn9z/bScbYWy105Q5d9Czz+rapgv
UtTvM+F3xMaZ3qnZve4Fq/hTFqDryE3tK5IZlg4Hxd6a3EqSAC7SfsPeb1PkIQwZ
JAf/bMuBDuTHSqkBf0N6o9CtXDNW89tgYyfFgZsjLZOAi7hYXdfLnh5kt7LIRLuY
llemikGTiHEEb0UK7xGhDgV8xoP9J7VsyXtVyCOgMvAmNXXVJeqiKTW7IzSnzT61
W60bAAozLq7fEoGNTceb/F/2S9wM+D6rpgNMFR2Et3KrDJnPw0JLVQE6ElM8WY9B
PPTpAgMBAAGjYDBeMB8GA1UdIwQYMBaAFNF/U3gOtDSFLeIOXr7atR5YkApWMB0G
A1UdDgQWBBROgcFVyOu9YJ9w7R8EVxj094QigjAMBgNVHRMBAf8EAjAAMA4GA1Ud
DwEB/wQEAwIHgDANBgkqhkiG9w0BAQsFAAOCAQEAB/EmpcQSCzIRlSWxVIr+bTis
sQihTnjVPaIMQ4SpMvgG9Xih/vRwhY2b17r4iFBBoPLBUQu8qhLSm0EHhTppU+Zd
QwJsRN8u2K2xnenbcGLOX1dcs3BjHVFpcqzYiYdWXse64zIL0oV32dibELOXS3Nq
IUZxmagG1TrfFsEDcosuIgNul7NowL5byQCi/dzyRftN/MdVJ9h0A026G0BBSPHJ
edViKMKkQ/hIImqgc4swKDPTo77a1UZcgIXkwxRE85U2fs6kLX8urrq1hiNgXbhB
rbquK1IUYfkYP0LRZXcM9rZQf8XQBJLttZczjsen8/R9koc0sfW7zHlPR3EixA==
-----END CERTIFICATE-----

)KEY";

// Device Private Key
static const char privatePemKey[] PROGMEM = R"KEY(
-----BEGIN RSA PRIVATE KEY-----
MIIEowIBAAKCAQEArt+HhoFyRLO6/AGGpEtniq53FEUwzV4vu+QDUgSSEtSoXeHQ
aXSln45/c/20nG2FstdOUOXfQs8/q2qYL1LU7zPhd8TGmd6p2b3uBav4Uxag68hN
7SuSGZYOB8XemtxKkgAu0n7D3m9T5CEMGSQH/2zLgQ7kx0qpAX9DeqPQrVwzVvPb
YGMnxYGbIy2TgIu4WF3Xy54eZLeyyES7mJZXpopBk4hxBG9FCu8RoQ4FfMaD/Se1
bMl7VcgjoDLwJjV11SXqoik1uyM0p80+tVutGwAKMy6u3xKBjU3Hm/xf9kvcDPg+
q6YDTBUdhLdyqwyZz8NCS1UBOhJTPFmPQTz06QIDAQABAoIBAHM0UY1cJBJ8sXep
7bC4+GqVi19wQblaDmpiyYkz96OuM4k95dZDgYU/FFouztirEoSCcVa8Sbwy2sgr
/jxW/m7jp+sGWKwv04feJzvXUk4yvyr2F9hiduqiP3YBdihhbkMHGdGUr6cPgp3s
45KAeUBdxSetzyw31GVnGOZnm2dJGsQbOyDMn0Sl23LGc+teSF6FWB/C+Q6LlDzG
hSNxNDPg1fYauG7JmDjZ2JcAshFCv0e0f1LKC/IW8WTWJgpUJxPxaZ7Ym7U1MaEd
JS2kUXAhvKX5nhU4V2fuh7DUpyyp9Pxya+9DU4aotjV6hTKWnZpN8mWXvbaDiLif
0EJnN/ECgYEA1NoPPLiZMzfpTe2AXweQ4zdfuSgM2PEsA7R6bG+alXReOe43y5L5
Ts97+ufWnAAvy7viZrguNYG9r641JD8auADywQ5WNLqIOdjZGh6vnUIJYDpVxWEa
EwpCE0TseLRqpnboubap3ZCL0QOFRYKVRaRC83XoU+V8KFb7NyiDn30CgYEA0lKR
vmUk9u9jF0pPs2jJGUCfHZ5TSXBQ+QIj19BwNROox1AF6lbeEccCKVq8g4vkva7+
q7xD25pg5OadnHcOZm37EsJ+K1hRpv7AZtCX0xSu18u8NSzjz8T/k0IQcWE9Q8FD
803e0nFmcOFRg9UwfVgHBM7y52/Fb1x/fWusPt0CgYAJDGQNbsOEe+H9IBWO9KFi
J154Xo19Q3NdjPSPW4c0x0eiKtBQZSUYBpX8qpJi+1cM0CXlu/qq56MKrtsveUdH
fzLSpitf/nYAD4nbJGbOxpnjb1dqFqCnIo6AhzOORg0qmm92RZ+e/aAT+JRXyh9S
YJNACkkgF+dZR6lSRPSjeQKBgQCI+CziCYyhJBUsPcDZ3gvY3jTBXTJwSSzyfWpK
SfxJFJx98KdpjVjhyvClbeoG4Tv+K9C+lsS6Zzdi2q0aEI7YNhekh/US1VmXn6A9
VOsLFBUphAz1tF5IGMs4Y3XB4Mm9LkfKU5QhCKjmKlajvFrvxjwL7ZpTDn9NnreI
Pm7WIQKBgBbtFQXEcdGdpPTuEGC76ttX6YLL7RtPOWCgpsBAOHbo28B9gEgDJRDz
4tnDxq3uFauH0pip7rWlCXC+jtveP8kH4wgBlxrDC2Iiyz3rkAa5pG5LwPaHo8FM
eCnuKrsayqwJ/H03XAMFHrBYWUOwdxz0GR6G6mnvs9HyQDiQbOgn
-----END RSA PRIVATE KEY-----

)KEY";


static const char rootCA[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF
ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6
b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL
MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv
b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj
ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM
9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw
IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6
VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L
93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm
jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC
AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA
A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI
U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs
N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv
o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU
5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy
rqXRfboQnoZsG4q5WTP468SQvvG5
-----END CERTIFICATE-----
)EOF";


WiFiClientSecure wiFiClient;
void msgReceived(char* topic, byte* payload, unsigned int len);
PubSubClient pubSubClient(awsEndpoint, 8883, msgReceived, wiFiClient);

void setup() {
    Serial.begin(115200);
    mySerial.begin(115200, SERIAL_8N1, 16, 17);  // RX=GPIO16, TX=GPIO17
    Serial.println("ESP32 AWS IoT Example");
    WiFi.begin(ssid, password);
    WiFi.waitForConnectResult();
    Serial.println("WiFi connected");

    wiFiClient.setCACert(rootCA);
    wiFiClient.setCertificate(certificatePemCrt);
    wiFiClient.setPrivateKey(privatePemKey);
}

void loop() {
    pubSubCheckConnect();

    if (mySerial.available()) {
        String data = mySerial.readStringUntil('\n'); // Read data from STM32
        Serial.println("Received from STM32: " + data);

       // Create a JSON object
        StaticJsonDocument<200> jsonDoc;
        jsonDoc["data"] = data; // Add the received data to the JSON object

        // Convert the JSON object to a string
        char jsonBuffer[200];
        serializeJson(jsonDoc, jsonBuffer);

        // Publish the JSON data to AWS IoT
        if (pubSubClient.connected()) {
            boolean rc = pubSubClient.publish("outTopic", jsonBuffer);
            Serial.print("Published, rc="); Serial.println(rc ? "OK" : "FAILED");
        }
    }
    pubSubClient.loop();
}


void msgReceived(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message received on "); Serial.print(topic); Serial.print(": ");
    for (int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
    }
    Serial.println();
}

void pubSubCheckConnect() {
    if (!pubSubClient.connected()) {
        Serial.print("PubSubClient connecting to: "); Serial.print(awsEndpoint);
        while (!pubSubClient.connected()) {
            Serial.print(".");
            pubSubClient.connect("ESP32_SENSOR_Thing");
            delay(1000);
        }
        Serial.println(" connected");
        pubSubClient.subscribe("inTopic");
    }
}
