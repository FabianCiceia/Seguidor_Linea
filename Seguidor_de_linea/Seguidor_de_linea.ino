// Definir los pines de los sensores
const int sensorIzquierda = 16;        // Sensor izquierdo (costado)
const int sensorCentralIzquierda = 5;  // Sensor central izquierda
const int sensorCentralDerecha = 14;   // Sensor central derecha
const int sensorDerecha = 12;          // Sensor derecho (costado)

// Definir los pines de los motores
const int motorIzquierdo = 4;  // Motor izquierdo
const int motorDerecho = 0;    // Motor derecho

// Variables de control PID
float kp = 60.0;  // Ganancia proporcional
float ki = 5.0;   // Ganancia integral
float kd = 10.0;  // Ganancia derivativa

// kp (Proporcional): Aumentar si el robot responde demasiado lento. Disminuir si oscila mucho.
// ki (Integral): Aumentar para corregir pequeños errores acumulados. No es siempre necesario.
// kd (Derivativo): Aumentar para suavizar la corrección y evitar sobrecorrección.

int error = 0;
int prevError = 0;
int integral = 0;
int derivativo = 0;

uint8_t velocidadBase = 200;  // Velocidad base
uint8_t velocidadMax = 255;   // Velocidad máxima de los motores

void setup() {
  // Configurar los pines de los sensores como entradas
  pinMode(sensorIzquierda, INPUT);
  pinMode(sensorCentralIzquierda, INPUT);
  pinMode(sensorCentralDerecha, INPUT);
  pinMode(sensorDerecha, INPUT);

  // Configurar los pines de los motores como salidas
  pinMode(motorIzquierdo, OUTPUT);
  pinMode(motorDerecho, OUTPUT);

  // Iniciar la comunicación serie para depuración
  Serial.begin(115200);
}

void loop() {
  // Leer los valores de los sensores
  int estadoIzquierda = digitalRead(sensorIzquierda);
  int estadoCentralIzquierda = digitalRead(sensorCentralIzquierda);
  int estadoCentralDerecha = digitalRead(sensorCentralDerecha);
  int estadoDerecha = digitalRead(sensorDerecha);

  if (estadoIzquierda == HIGH && estadoCentralIzquierda == HIGH && estadoCentralDerecha == HIGH && estadoDerecha == HIGH) {
    detenerMotores();
  } else {
    // Determinar la posición del robot respecto a la línea
    error = calcularError(estadoIzquierda, estadoCentralIzquierda, estadoCentralDerecha, estadoDerecha);

    // Control PID
    integral += error;
    derivativo = error - prevError;

    int ajuste = (kp * error) + (ki * integral) + (kd * derivativo);
    prevError = error;

    // Calcular las velocidades de los motores basadas en el ajuste PID
    int velocidadMotorIzquierdo = velocidadBase + ajuste;
    int velocidadMotorDerecho = velocidadBase - ajuste;

    // Limitar las velocidades dentro del rango [0, velocidadMax]
    velocidadMotorIzquierdo = constrain(velocidadMotorIzquierdo, 0, velocidadMax);
    velocidadMotorDerecho = constrain(velocidadMotorDerecho, 0, velocidadMax);

    // Aplicar las velocidades a los motores
    analogWrite(motorIzquierdo, velocidadMotorIzquierdo);
    analogWrite(motorDerecho, velocidadMotorDerecho);

    // Mostrar los valores en el monitor serie
    Serial.print("Izquierda: ");
    Serial.print(estadoIzquierda);
    Serial.print(" Central Izquierda: ");
    Serial.print(estadoCentralIzquierda);
    Serial.print(" Central Derecha: ");
    Serial.print(estadoCentralDerecha);
    Serial.print(" Derecha: ");
    Serial.print(estadoDerecha);
    Serial.print(" Error: ");
    Serial.print(error);
    Serial.print(" Ajuste: ");
    Serial.println(ajuste);
  }


  delay(50);  // Espera antes de la siguiente lectura
}

// Función para calcular el error en función de los sensores
int calcularError(int izq, int centIzq, int centDer, int der) {
  // Si ambos sensores centrales detectan la línea, el robot sigue recto
  if (centIzq == LOW && centDer == LOW) {
    return 0;  // No hay error, seguir recto
  }

  // Casos para girar a la izquierda:
  if (centIzq == LOW && izq == HIGH) {
    return -1;  // El central izquierdo detecta, giro leve a la derecha
  } else if (centIzq == LOW && izq == LOW) {
    return -2;  // El central izquierdo y el izquierdo detectan, giro moderado a la derecha
  } else if (izq == LOW && centIzq == HIGH) {
    return -3;  // Solo el izquierdo detecta, giro brusco a la derecha
  }

  // Casos para girar a la derecha:
  if (centDer == LOW && der == HIGH) {
    return 1;  // El central derecho detecta, giro leve a la izquierda
  } else if (centDer == LOW && der == LOW) {
    return 2;  // El central derecho y el derecho detectan, giro moderado a la izquierda
  } else if (der == LOW && centDer == HIGH) {
    return 3;  // Solo el derecho detecta, giro brusco a la izquierda
  }

  // Si no se detecta la línea, detener el robot
  return 0;
}
void detenerMotores() {
  analogWrite(motorIzquierdo, 0);
  analogWrite(motorDerecho, 0);
  Serial.println("Deteniendo motores");
}
