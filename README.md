# embedded-astar

[![crates.io](https://img.shields.io/crates/v/embedded-astar.svg)](https://crates.io/crates/embedded-astar)
[![docs.rs](https://docs.rs/embedded-astar/badge.svg)](https://docs.rs/embedded-astar)
[![Licence](https://img.shields.io/badge/licence-GPL--2.0--or--later-blue.svg)](#licence)
[![no_std](https://img.shields.io/badge/no__std-compatible-success.svg)](#)

Bibliothèque de recherche de chemin A\* `no_std` pour systèmes embarqués.

Conçue pour les microcontrôleurs et les environnements à ressources limitées, cette crate
fournit une implémentation complète d'A\* avec **zéro allocation sur le tas**, reposant
uniquement sur des structures de taille fixe allouées sur la pile via [`heapless`].

---

## Fonctionnalités

| Fonctionnalité | Détail |
|---|---|
| Compatible `no_std` | Pas de tas, pas de crate `alloc` requis |
| Zéro code `unsafe` | `#![deny(unsafe_code)]` appliqué |
| Dimensions const-génériques | Taille de grille fixée à la compilation |
| Heuristique de Manhattan | Optimale pour le déplacement 4 directions |
| Sécurité mémoire | Open set plein géré sans panique |
| Déterministe | Pas de dispatch dynamique, pas d'aléatoire |

---

## Installation

Ajouter dans votre `Cargo.toml` :

```toml
[dependencies]
embedded-astar = "0.1"
```

Pour les cibles embarquées sans bibliothèque standard, assurez-vous que votre crate
déclare `#![no_std]`. `embedded-astar` ne nécessite aucune configuration supplémentaire.

---

## Démarrage rapide

```rust
use embedded_astar::{trouver_chemin, Point};

// Grille 8×8 : true = obstacle, false = case libre
// Indexée sous la forme grille[y][x]
let mut grille = [[false; 8]; 8];
grille[2][1] = true; // obstacle en (x=1, y=2)
grille[2][2] = true;
grille[2][3] = true;

let depart  = Point::nouveau(0, 0);
let arrivee = Point::nouveau(7, 7);

// Paramètres de type : <LARGEUR, HAUTEUR, CAPACITE_MAX>
match trouver_chemin::<8, 8, 64>(depart, arrivee, &grille) {
    Some(chemin) => {
        for point in chemin.iter() {
            // Déplacer le robot/agent vers point.x, point.y
        }
    }
    None => {
        // Aucun chemin trouvé (bloqué, hors bornes, ou open set saturé)
    }
}
```

---

## API

### `trouver_chemin`

```rust
pub fn trouver_chemin<
    const LARGEUR: usize,
    const HAUTEUR: usize,
    const CAPACITE_MAX: usize,
>(
    depart:  Point,
    arrivee: Point,
    grille:  &[[bool; LARGEUR]; HAUTEUR],
) -> Option<heapless::Vec<Point, CAPACITE_MAX>>
```

Exécute A\* sur une grille booléenne d'obstacles statique.

**Retourne :**
- `Some(chemin)` — liste ordonnée de [`Point`]s du départ à l'arrivée (inclus)
- `None` — si aucun chemin n'existe, qu'un point est bloqué ou hors bornes,
  ou que le chemin dépasse la capacité `CAPACITE_MAX`

### `Point`

```rust
pub struct Point { pub x: i32, pub y: i32 }

impl Point {
    pub const fn nouveau(x: i32, y: i32) -> Self;
}
```

---

## Convention de grille

```
  x → 0  1  2  3
y
↓
0     .  .  .  .
1     .  #  .  .    # = obstacle  (grille[1][1] = true)
2     .  .  .  .
3     .  .  .  .
```

- `grille[y][x]` — Y est le premier index (rangée principale)
- `true`  → obstacle (case infranchissable)
- `false` → case libre (franchissable)

---

## Réglage des capacités

Les trois génériques constants contrôlent **toute l'utilisation mémoire** —
il n'y a aucune allocation dynamique à quelque moment que ce soit.

| Paramètre      | Rôle                             | Recommandation           |
|----------------|----------------------------------|--------------------------|
| `LARGEUR`      | Largeur de la grille (colonnes)  | = largeur de votre carte |
| `HAUTEUR`      | Hauteur de la grille (lignes)    | = hauteur de votre carte |
| `CAPACITE_MAX` | Capacité open set et chemin      | `≥ 2 × √(L × H)`        |

**Estimation de l'utilisation de la pile** (approximatif, sans padding d'alignement) :

```
ensemble fermé  : LARGEUR × HAUTEUR      octets   (tableau de bool)
parents         : LARGEUR × HAUTEUR × 9  octets   (Option<Point>)
open set        : CAPACITE_MAX × 16      octets   (tableau de Noeud)
chemin          : CAPACITE_MAX × 8       octets   (tableau de Point)
```

Pour une grille 32×32 avec `CAPACITE_MAX = 128` :

```
ensemble fermé  :  1 024 o
parents         :  9 216 o
open set        :  2 048 o
chemin          :  1 024 o
────────────────────────────
Total           ≈ 13 312 o  (~13 Ko)
```

### Configurations typiques

| Cas d'usage               | `LARGEUR` | `HAUTEUR` | `CAPACITE_MAX` | Pile    |
|---------------------------|-----------|-----------|----------------|---------|
| Petit labyrinthe (robot)  | 16        | 16        | 64             | ~3 Ko   |
| Carte moyenne (drone)     | 32        | 32        | 128            | ~13 Ko  |
| Grande carte (AGV)        | 64        | 64        | 256            | ~52 Ko  |

---

## Modèle de déplacement

Par défaut, le déplacement est **4-directionnel** (directions cardinales uniquement) :

```
     ↑
  ←  ·  →
     ↓
```

Tous les déplacements ont un coût uniforme de 1. Le déplacement diagonal n'est pas
inclus par défaut afin de garantir l'admissibilité de l'heuristique (distance de
Manhattan). Pour activer le déplacement 8-directionnel, utiliser la distance de
Chebyshev comme heuristique et ajouter `(±1, ±1)` au tableau des directions.

---

## Détails algorithmiques

L'implémentation suit le graphe de recherche A\* standard :

1. **Open set** — un `heapless::Vec` de nœuds dont le meilleur `cout_f` est sélectionné
   via scan linéaire (O(n), optimal pour les petits n typiques des grilles embarquées)
2. **Ensemble fermé** — une grille `bool` plate (accès O(1), mémoire constante)
3. **Carte des parents** — une grille plate `Option<Point>` pour la reconstruction du chemin
4. **Heuristique** — distance de Manhattan `|Δx| + |Δy|` (admissible et consistante)
5. **Reconstruction** — remonte la carte des parents de l'arrivée au départ, puis inverse

Aucun `BinaryHeap`, aucun `HashMap`, aucun `Vec` de `std`.

---

## Utilisation en no_std

Dans la racine de votre crate (`main.rs` ou `lib.rs`) :

```rust
#![no_std]

use embedded_astar::{trouver_chemin, Point};
```

Pour les cibles sans gestionnaire de panique, en ajouter un :

```rust
use core::panic::PanicInfo;

#[panic_handler]
fn panique(_: &PanicInfo) -> ! {
    loop {}
}
```

---

## Contribuer

Les contributions sont les bienvenues ! Ouvrir des issues ou des pull requests sur
[GitHub](https://github.com/jorgeandrecastro/embedded-astar).

Avant de soumettre :
- Exécuter `cargo test`
- Exécuter `cargo clippy -- -D warnings`
- Exécuter `cargo fmt --check`

---

## Licence

Ce programme est un logiciel libre distribué selon les termes de la
**Licence Publique Générale GNU version 2** (GPL-2.0-or-later).

Voir le fichier [LICENSE](LICENSE) ou consulter <https://www.gnu.org/licenses/gpl-2.0.html>.