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
