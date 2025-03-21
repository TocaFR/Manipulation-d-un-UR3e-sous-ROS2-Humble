# Programme exemple

Téléchargez et lancez [le programme d'exemple](example_move.py) afin de tester le bon fonctionnement de python et du robot avec la commande suivante (il faut que le terminal soit dans le répertoire du script python afin de l'utiliser).

Avant toute chose, rendez le fichier python exécutable :

```
chmod +x example_move.py
```

Enfin lancez le programme avec :

```
python3 example_move.py
```

Dans ce programme d'exemple nous y trouverons directement les trajectoires, cependant lorsqu'il y a une multitude de trajectoires dans un programme, le code finira par devenir trop long.

# Notre programme de palettisation

C'est pour cela que nous avons créé un programme de palettisation faisant appel à un fichier .xml contenant nos trajectoires, ainsi qu'un sous programme manipulant la sortie numérique 0 du robot. Ce qui nous permet de manipuler l'ouverture et la fermeture de la pince RG2 de notre robot en utilisant un thread dans notre programme robot (en plus de l'instruction de contrôle externe). Nous avons également inséré la notion de programmation orienté objet (classes d'objet) afin de simplifier la rédaction du code.

Vous pouvez éxecutez le programme de la même manière qu'avant : 

```
python3 programme_palettisation.py
```

Le sous programme ainsi que le fichier .xml seront automatiquement appelés par le script, pas besoin de s'en occuper dans d'autres terminaux.

# Quelques explications

## **Fichier XML**

Dans le fichier .xml, nous aurons donc différentes trajectoires contenant différents points ayant pour attribut :
- L'état de la pince
- Les vitesses angulaires de chaque axe
- Les positions angulaires de chaque axe
- À partir de quel valeur de temps (en secondes) le robot doit atteindre la position

![image](https://github.com/user-attachments/assets/13c5d24e-2dad-42ae-98dc-e0d66e05c4df)

## **Thread robot pour manipuler la pince**

- Dans le programme Robot, ajoutez un thread en dessous de l'instruction de contrôle externe
- Ajoutez une condition Si ... Sinon
- Si *digital_output[0] = True* Alors *RG2(60)* (Pince ouverte)
- Sinon Si *digital_output[0] = False* Alors *RG2(0)* (Pince fermée)

Si vous n'utilisez pas de pince, le robot pourra dans tous les cas effectuer le programme python puisqu'on ne fait que manipuler une sortie externe.
