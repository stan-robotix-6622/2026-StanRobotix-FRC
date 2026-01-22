# Stan-Robotix-Source ![star](https://img.shields.io/github/stars/PragmaCube/Stan-Robotix-Source)
Dépôt officiel de l'équipe [Stan Robotix 6622](https://stanrobotix6622.com/) pour l'édition 2026 de [FRC](https://www.firstinspires.org/robotics/frc)

## Conventions de structure
- Branches :`Année-StanRobotix-Compétition-SousSytème/Commande`
- Répertoires : `Année_StanRobotix_Compétition/Projet`

## Les conventions de code
- Programmer en anglais
- Variables dans les méthodes :
  - Préfixe i (input) pour les variables d'entrée (e.g. iMotor)
  - Préfixe o (output) pour les variables de sorties (e.g. oAngle)
- Variables locales :
  - Préfixe w (work) (e.g. wSpeed)
- Dans une classe :
  - Préfixe m (my) pour les attributs (e.g. mAccelerometer)
  - Le nom des méthodes commence par une minuscule puis on utilise des majuscules pour les autres mots (e.g. setVoltage())
- Nomenclature :
  - Subsystems: `Sub` suivit du type de subsystème (e.g. Drivetrain, Intake, IMU...)
  - Commandes: ce que fait la commande (e.g. SetElevatorPosition, ActivateShooter, ControlIntake...)
- Préfixe k (constant) pour les variables constantes (e.g. kPi, kLeftMotorCanID...)
- Mettre des commentaires mais ne pas en abuser // Ceci est un abus

## Rappel des commandes de base de Git
```
git clone
git status
git add *
git add "NomDuFichier"
git rm "NomDuFichier"
git commit -m "MessageQuiADuSens"
git push
git checkout NomDeLaBranche
git branch NomDeLaBranche
git pull
git fetch
gitk
```
## Gestion
La gestion du dépôt (modification du README ou modifications majeures) est laissée au chef de l'équipe et aux mentors.


Chef d'équipe : Timothée Laberge<br>
Mentors : Raphaël Pothier, André Wojcik et Micha Reneault<br>

Contacts :<br>
timothee.laberge@stanislas.qc.ca <br>
raphael.pothier@stanislas.ca <br>
wojcik.andre2.0@gmail.com <br>
micha.reneault@gmail.com <br>

2026