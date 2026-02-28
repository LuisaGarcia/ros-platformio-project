/**************************************************************
 *  PROGRAMA: Control de 4 Servomotores para brazo robótico controlado por un 
 * nodo de ros2 *  
 *  Este programa permite controlar 4 servos enviando comandos
 *  por el monitor serial con el siguiente formato:
 *
 *      Letra:numero
 *
 *  Ejemplos:
 *      P:90
 *      C:45
 *      H:120
 *      R:0
 *
 *  Donde:
 *      P = Servo conectado al pin 3
 *      C = Servo conectado al pin 5
 *      H = Servo conectado al pin 6
 *      R = Servo conectado al pin 9
 *
 *  El número representa el ángulo del servo (0 a 180 grados).
 *  P Pinza
 *  C Codo
 *  H Hombro
 *  R Base rotor
 **************************************************************/

#include <Arduino.h>   // Librería base de Arduino
#include <Servo.h>     // Librería para controlar servomotores

// Se crean 4 objetos tipo Servo
Servo servoP;
Servo servoC;
Servo servoH;
Servo servoR;

void setup() {

  // Inicia la comunicación serial a 9600 baudios
  // Permite enviar y recibir datos desde el computador
  Serial.begin(9600);

  // Se conectan los servos a los pines digitales correspondientes
  servoP.attach(3);  // Servo P conectado al pin 3
  servoC.attach(5);  // Servo C conectado al pin 5
  servoH.attach(6);  // Servo H conectado al pin 6
  servoR.attach(9);  // Servo R conectado al pin 9

  // Mensaje inicial para el usuario
  Serial.println("Listo. Envie comandos en formato Letra:numero (P,C,H,R).");
}

void loop() {

  // Verifica si hay datos disponibles en el puerto serial
  if (Serial.available()) {

    // Lee la cadena recibida hasta encontrar un salto de línea
    String data = Serial.readStringUntil('\n');

    // Elimina espacios o caracteres invisibles como \r
    data.trim();

    // Muestra en pantalla lo que se recibió (para depuración)
    Serial.print("Recibido: ");
    Serial.println(data);

    // Busca la posición del carácter ':' en el texto recibido
    int separator = data.indexOf(':');

    // Verifica que el formato sea correcto (que exista ':')
    if (separator > 0) {

      // Extrae la primera letra (posición 0)
      char letra = data.charAt(0);

      // Extrae el número después del ':'
      // substring(separator + 1) toma desde después del ':' hasta el final
      int numero = data.substring(separator + 1).toInt();

      // Muestra los valores interpretados
      Serial.print("Letra = ");
      Serial.println(letra);
      Serial.print("Numero = ");
      Serial.println(numero);

      /**************** VALIDACIONES ****************/

      // Validar que la letra sea una de las permitidas
      if (letra != 'P' && letra != 'C' && letra != 'H' && letra != 'R') {
        Serial.println("ERROR: Letra no válida. Use P, C, H o R.");
        return;  // Sale del loop actual sin mover ningún servo
      }

      // Validar que el ángulo esté entre 0 y 180 grados
      if (numero < 0 || numero > 180) {
        Serial.println("ERROR: El numero debe estar entre 0 y 180.");
        return;  // Sale sin mover el servo
      }

      /**************** MOVIMIENTO DEL SERVO ****************/

      // Dependiendo de la letra recibida,
      // se mueve el servo correspondiente
      switch (letra) {

        case 'P':
          servoP.write(numero);  // Mueve servoP al ángulo indicado
          break;

        case 'C':
          servoC.write(numero);  // Mueve servoC al ángulo indicado
          break;

        case 'H':
          servoH.write(numero);  // Mueve servoH al ángulo indicado
          break;

        case 'R':
          servoR.write(numero);  // Mueve servoR al ángulo indicado
          break;
      }

      // Mensaje de confirmación
      Serial.print("OK -> Servo ");
      Serial.print(letra);
      Serial.print(" movido a ");
      Serial.println(numero);

    } 
    else {
      // Si no se encontró ':' el formato es incorrecto
      Serial.println("ERROR: Formato incorrecto. Use Letra:numero");
    }
  }
}
