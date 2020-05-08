#include <GL/glut.h>
#include "./armadillo/include/armadillo"

using namespace std;
using namespace arma;

#define PI 3.14159265359f // Pour les angles en radians

/** Prototypes des méthodes ******************************************/

void drawObject();              // Dessin du bras articulé
void computePivotCoordinates(); // (Re-)Calcul des coordonnées des Pivots
float norm(vec P);                      // Calcul de la norme d'un vecteur
void computeCoordinates(vec new_Cible); // Calcul des variations et mise à jour
vec get_New_Cible(int x, int y);        // Renvoie une cible valable à partir des entiers
int reverse_Cible_X(vec C);             // Retrouve le x entier à partir des coordonnées
int reverse_Cible_Y(vec C);             // Retrouve le y entier à partir des coordonnées
void myinit();                          // Initialisation des données

// Méthodes d'interaction GLUT
void mouse(int button, int state, int x, int y); // gestion des clics souris
void motion(int x, int y);                       // gestion des mouvements de la souris
void parsekey(unsigned char key, int x, int y);  // gestion des touches du clavier
void parsekey_special(int key, int x, int y);    // gestion des touches speciales
void myReshape(int w, int h);                    // gestion du redimensionnement
void display();                                  // gestion de l'affichage