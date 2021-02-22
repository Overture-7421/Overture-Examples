#include "Skyline.h"
#include "frc2/command/CommandScheduler.h"
#include <frc/smartdashboard/SmartDashboard.h>
#define M_PI 3.14159265358979323846



/**
 *  El objeto va a ser controlar la velocidad real de los motores, hasta ahorita solo les hemos estado un voltaje constante.
 *  Esto funciona bien hasta cierto punto, pero puede causar problemas cuando queremos hacer cosas autonomas; ¿Cuanto avanzó el robot?
 * ¿Qué tan rapido está yendo? Ahorita no podemos responder estas preguntas, solo le decimos al motor, ve a 0.4 (40% de la batería).
 * 
 * 
 *  Poder responder esas preguntas, y por consiguiente controlar al motor de manera más completa, haremos lo siguiente.
 * 
 *  1. Graficar las velocidades de los motores
 *  2. Intentar controlar la velocidad del motor por porcentaje, introduciendo manualmente las velocidades usando la Dashboard
 *  3. Entender en que unidades están y transformarlas a radianes por segundo
 *     
 *  Hasta aqui, deberían de llegar a algo similar a esto:
 *      https://owncloud.porebazu.tech/s/2TqsytGqvOKbsCn
 * 
 * 
 *  Esto se hará hasta el miercoles probablemente.
 *  4. Intentar controlar la velocidad del motor usando un PID.
 *  
 *      
 * */


Skyline::Skyline() : TimedRobot() {}

void Skyline::RobotInit() {
   /**
    * Configuramos el motor para que use el Quadencoder como sensor para la velocidad.
    * 
    * Los QuadEncoders son relativos, es decir, cuando el robot enciende, ese es su cero. Si se mueven incrementan o disminyuen. Esto quiere decir que no saben donde están realmente, solo saben
    *   que tanto se han movido desde que empezaron a contar. Los encoders son utiles para controlar velocidades.
    * 
    * 
    * Los Potenciometros son abosultos, es decir, su cero siempre es el mismo. Son utiles para controlar torretas, angulo de brazos, o cualquier cosa que ocupemos una referencia absoluta.
    * 
    * Una manera de diferenciarlos es considerar que pasaría si encendemos el robot, giramos una llanta y lo apagamos. ¿Que pasaría la siguiente vez que lo encendamos?
    * 
    *   Si asumimos que la primera vez que lo prendamos el encoder y potenciometro dieron cero, despues de moverlo ambos podrían dar el mismo resultado.
    *   Si apagamos y volvemos a prender el robot, el potenciometro seguiría dando el mismo valor anterior, pero el encoder volvería a dar cero. 
    * */
    leftMotor.setFeedbackMode(MotorFeedbackMode::QuadEncoder);
    rightMotor.setFeedbackMode(MotorFeedbackMode::QuadEncoder);

}


void Skyline::RobotPeriodic() { 
    /**
     * Podemos usar la SmartDashboard para poder cambiar valores desde la interfaz, en lugar de directo en el código. Esto hace que las calibraciones o pruebas de diferentes valores sea
     * mucho más rapido.
     * 
     * Se pueden poner 3 cosas diferentes en la Dashboard sencillamente.
     * 
     *  frc::SmartDashboard::PutNumber -> Pone un número
     *  frc::SmartDashboard::PutString -> Pone una palabra
     *  frc::SmartDashboard::PutBoolean -> Pone un booleano
     * 
     *  Esas funciones ocupan dos parametros, el primero es el nombre que tendrá el valor en la Dashboard, el segundo es el valor en si mismo. 
     * 
     * Ahorita pondremos las velocidades de los motores en la Dashboard.
     * 
     * Usualmente para cosas de telemetría (Saber que esta haciendo el robot) usamos RobotPeriodic, queremos ver esa información sin importar en que modo de ejecución esté el robot.
     * */
    

    /**
     *  Si ponemos una / en el nombre, creara una subtabla.
     *  Esto se ocupa ahorita porque todos estan conectados a la misma Dashboard, si todos ponen un número llamado LeftVel e intentan usarlo, una persona cambiaría el de todos.
     * 
     * */
    frc::SmartDashboard::PutNumber("TankChassis0/LeftVel", leftMotor.getVelocity()); 
    frc::SmartDashboard::PutNumber("TankChassis0/RightVel", rightMotor.getVelocity());

    /**
     *  Estos valores están en -> Pulsos de encoder / segundo. 
     * 
     *  Usualmente queremos transformar esta velocidad a radianes por segundo, para que podamos usar matemáticas más formales.
     * 
     *  Los encoders tienen cierta cantidad de pulsos por cada revolución. Puede ser que mi encoder de 100 pulsos por revolución del motor. 
     *  Si quiero saber cuantas vueltas ha dado puedo hacer esto:
     *      vueltas = encoderPos / 100.0; 
     *  Donde   encoderPos es en que posición del encoder estamos ahorita.
     * 
     * 
     *  Es importante considerar donde está el encoder. Usualmente están directos en el motor, nos dicen cuanto se ha movido el motor en especifico.
     *  Esto puede causar problemas cuando se usan reducciones.
     * 
     *  Si conectamos un motor directo a las llantas del chassis, sería casi imposible que el robot avanzara. El robot pesa mucho y el motor va muy rapido, no puede generar el torque necesario.
     *  Mecanicamente eso se soluciona usando una reducción, hacen que por cada vuelta que de el motor la llanta gire menos. Podría ser que por cada 10 vueltas del motor, la llanta da una sola.
     * 
     *  Esto incrementa en 10 veces el torque generado en la llanta por el motor, pero reduce la velocidad 10 veces.
     * 
     *  A nosotros nos importa la velocidad de la llanta, no la del motor, entonces tenemos que considerar la reducción al hacer nuestros cálculos.
     * 
     *  Los robots simulados tienen 420 pulsos de encoder por vuelta del motor, y el motor tiene una reducción de 10 a 1 a la llanta.
     *  
     *  Podemos obtener de esa información cuantos pulsos del encoder representan una vuelta del motor.
     * 
     *      pprWheel = pprMotor * gearRatio; 
     * 
     *  Donde   pprWheel es pulsos por revolución de la llanta
     *          pprMotor es pulsos por revolución del motor
     *          gearRatio cuantas veces tiene que girar el motor para que la llanta gire una vez
     *      
     *  
     *  Entonces, para obtener la velocidad del motor en revoluciones por segundo, podemos hacer lo siguiente.
     * 
     * 
     *      revsPerSecond = motor.getVelocity() / pprWheel;
     * 
     *  Una vez tenemos esto, es cuestión de multiplicar por 2 PI. Gracias a que una revolución equivale a 2 PI, esto nos transforma la velocidad a radianes cada segundo.
     *  
     *  vel = revsPerSecond * 2 * M_PI;
     * 
     *  Vamos a graficar esta velocidad, estoy uniendo todos los calculos en uno.
     * 
     * */
    const double codesPerRevWheel = 420.0 * 10.0;
    const double leftMotorVel = leftMotor.getVelocity() / codesPerRevWheel * 2 * M_PI;
    const double rightMotorVel = rightMotor.getVelocity() / codesPerRevWheel * 2 * M_PI;

    frc::SmartDashboard::PutNumber("TankChassis0/LeftVelRads", leftMotorVel); 
    frc::SmartDashboard::PutNumber("TankChassis0/RightVelRads", rightMotorVel);

    /**
    *   Podemos ir aun más adelante, y pasar esa velocidad a metros por segundo.
    * 
    *   Gracias a que tenemos la velocidad en radianes por segundo, es cuestion de multiplicarlo por el radio de la llanta. 
    *   Usamos llantas de 6 pulgadas de diametro. El radio en metros es 0.0762.
    *  
    * */ 

    const double wheelRadius = 0.0762;
    frc::SmartDashboard::PutNumber("TankChassis0/LeftVelMeters", leftMotorVel * wheelRadius); 
    frc::SmartDashboard::PutNumber("TankChassis0/RightVelMeters", rightMotorVel * wheelRadius);



}

void Skyline::DisabledInit() {}

void Skyline::DisabledPeriodic() {}

void Skyline::AutonomousInit() {
    /**
     *  Vamos a poner dos números, cada uno va a representar a que porcentaje queremos mover los motores.
     * 
     *  Usaremos valores iniciales de cero.
     * */

    frc::SmartDashboard::PutNumber("TankChassis0/LeftOut", 0.0);
    frc::SmartDashboard::PutNumber("TankChassis0/RightOut", 0.0);
}

void Skyline::AutonomousPeriodic() {
    /**
     *  Podemos obtener valores de la dashboard usando 
     * 
     *  frc::SmartDashboard::GetNumber -> Obtiene un número
     *  frc::SmartDashboard::GetString -> Obtiene una palabra
     *  frc::SmartDashboard::GetBoolean -> Obtiene un booleano1
     * 
     *  Esas funciones ocupan dos parametros, el primero es el nombre del valor que queremos obtener, el segundo es el valor por defecto, por si ese valor no se encuentra en la Dashboard.
     * 
     * 
     *  Debería de verse algo asi: https://owncloud.porebazu.tech/s/mJbraf4YcGsUPcu
     * */


    double leftOut = frc::SmartDashboard::GetNumber("TankChassis0/LeftOut", 0.0);
    double rightOut = frc::SmartDashboard::GetNumber("TankChassis0/RightOut", 0.0);
    leftMotor.set(leftOut); //Por ahorita no usaremos las ecuaciones que reciben velocidad linear y angular, para simplificar calibrar PIDs despues.
    rightMotor.set(rightOut);

}

void Skyline::TeleopInit() {


}

void Skyline::TeleopPeriodic() {

}

void Skyline::TestPeriodic() {}
