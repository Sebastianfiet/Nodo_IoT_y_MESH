#include "painlessMesh.h"
#include <Arduino_JSON.h>

#define MESH_PREFIX     "joserafaelparra"
#define MESH_PASSWORD   "planetavegetta777"
#define MESH_PORT       5555

/**
 * Class: RGBNode
 * Clase para manejar el nodo RGB.
 *
 * Attributes:
 * - redPin (int): El pin GPIO al que está conectado el led rojo el RGB.
 * - greenPin (int): El pin GPIO al que está conectado el led verde el RGB.
 * - bluePin (int): El pin GPIO al que está conectado el led azul el RGB.
 * - mesh (painlessMesh): Objeto de la red MESH para manejar la comunicación entre nodos.
 * - userScheduler (Scheduler): scheduler para manejar las tareas del nodo.
 *
 * Methods:
 * - setColor(int red, int green, int blue): Modifica el color del RGB.
 * - blinkErrorLED(): Parpadea el led rojo del RGB en caso de errores.
 * - receivedCallback(uint32_t from, String &msg): Función para procesar los mensajes provenientes de la red.
 */

Scheduler userScheduler;
painlessMesh mesh;

const int redPin = 13;    // Pin for Red color of RGB LED
const int greenPin = 12;  // Pin for Green color of RGB LED
const int bluePin = 14;   // Pin for Blue color of RGB LED

/**
 * Define los colores para el RGB.
 *
 * Parametros:
 * - red (int): La intensidad del color rojo (0-255).
 * - green (int): La intensidad del color verde (0-255).
 * - blue (int): La intensidad del color azul (0-255).
 */
void setColor(int red, int green, int blue) {
  ledcWrite(0, red);   // channel 0 controls Red
  ledcWrite(1, green); // channel 1 controls Green
  ledcWrite(2, blue);  // channel 2 controls Blue
}

/**
 * Función para hacer parpadear el LED durante 3 segundos en caso de error.
 */
void blinkErrorLED() {
  for (int i = 0; i < 6; i++) {  // 6 ciclos para un total de 3 segundos (500ms encendido, 500ms apagado)
    setColor(255, 0, 0);  // Rojo (indicando error)
    delay(500);           // Espera 500ms
    setColor(0, 0, 0);    // Apagar el LED
    delay(500);           // Espera 500ms
  }
}

/**
 * Función para manejar mensahes provenientes de la red MESH, los cuales se espera estén en un formato Json.
 * 
 * Parametros:
 * - from (uint32_t): ID del nodo que envía el mensaje.
 * - msg (String): El mensaje recibido.
 */
void receivedCallback(uint32_t from, String &msg) {
  Serial.printf("Mensaje recibido del nodo %u: %s\n", from, msg.c_str());

  // Parsear el mensaje JSON recibido
  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, msg);

  // Si hay un error en el formato JSON, activar el parpadeo del LED
  if (error) {
    Serial.print("Error deserializando JSON: ");
    Serial.println(error.c_str());
    blinkErrorLED();  // Parpadear LED en caso de error en el formato JSON
    return;
  }

  // Extraer la temperatura del mensaje
  float temperature = doc["temp"];
  Serial.printf("Temperatura recibida: %.2f\n", temperature);

  // Detectar si la temperatura es un dato erróneo (fuera de los rangos esperados)
  if (temperature < -10 || temperature > 60) {
    Serial.println("Temperatura fuera de rango. Activando parpadeo de error.");
    blinkErrorLED();  // Parpadear LED si la temperatura está fuera de los límites
    return;
  }

  // Cambiar el color del LED según la temperatura recibida
  if (temperature < 20) {
    setColor(0, 0, 255);  // Azul para temperaturas < 20
  } else if (temperature >= 20 && temperature <= 24) {
    setColor(0, 255, 0);  // Verde para temperaturas entre 20 y 24
  } else {
    setColor(255, 0, 0);  // Rojo para temperaturas > 24
  }
}

/**
 * Inicializa los pines del RGB, el objeto de la red MESH y las funciones.
*/
void setup() {
  Serial.begin(115200);

  // Configurar los pines del LED RGB
  ledcSetup(0, 5000, 8);  // Red LED en canal 0
  ledcSetup(1, 5000, 8);  // Green LED en canal 1
  ledcSetup(2, 5000, 8);  // Blue LED en canal 2
  ledcAttachPin(redPin, 0);
  ledcAttachPin(greenPin, 1);
  ledcAttachPin(bluePin, 2);

  // Inicializar la red mesh
  mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION);
  mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
  mesh.onReceive(receivedCallback);  // Establecer callback de recepción
}

/**
 * Actualiza la red mesh.
*/
void loop() {
  mesh.update();
}