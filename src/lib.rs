// embedded-astar : Algorithme A* no_std pour systèmes embarqués
// Copyright (C) 2024  Jorge Andre Castro
//
// Ce programme est un logiciel libre ; vous pouvez le redistribuer et/ou
// le modifier selon les termes de la Licence Publique Générale GNU publiée
// par la Free Software Foundation ; soit la version 2 de la Licence, soit
// (à votre choix) toute version ultérieure.
//
// Ce programme est distribué dans l'espoir qu'il sera utile, mais SANS
// AUCUNE GARANTIE ; sans même la garantie implicite de COMMERCIALISABILITÉ
// ou d'ADÉQUATION À UN USAGE PARTICULIER. Voir la Licence Publique Générale
// GNU pour plus de détails.
//
// Vous devriez avoir reçu une copie de la Licence Publique Générale GNU avec
// ce programme ; si ce n'est pas le cas, consultez <https://www.gnu.org/licenses/>.

//! # embedded-astar
//!
//! Bibliothèque de recherche de chemin A\* `no_std` pour systèmes embarqués.
//!
//! Conçue pour les microcontrôleurs et les environnements à ressources limitées,
//! cette crate fournit une implémentation complète d'A\* **sans allocation sur le tas**,
//! reposant uniquement sur des structures de taille fixe allouées sur la pile
//! grâce à [`heapless`].
//!
//! ## Fonctionnalités
//!
//! - **Compatible `no_std`** — aucun tas, aucun allocateur requis
//! - **Dimensions configurables** — `LARGEUR`, `HAUTEUR` et `CAPACITE_MAX` fixés à la compilation
//! - **Heuristique de Manhattan** — optimale pour les grilles à 4 directions
//! - **Sécurité mémoire** — dépassement de l'open set géré sans panique
//! - **Zéro code `unsafe`**
//!
//! ## Démarrage rapide
//!
//! ```rust
//! use embedded_astar::{trouver_chemin, Point};
//!
//! // Grille 8×8 : true = obstacle, false = case libre
//! // Indexée sous la forme grille[y][x]
//! let mut grille = [[false; 8]; 8];
//! grille[1][2] = true; // obstacle en (x=2, y=1)
//!
//! let depart  = Point::nouveau(0, 0);
//! let arrivee = Point::nouveau(7, 7);
//!
//! // Paramètres de type : <LARGEUR, HAUTEUR, CAPACITE_MAX>
//! if let Some(chemin) = trouver_chemin::<8, 8, 64>(depart, arrivee, &grille) {
//!     for p in chemin.iter() {
//!         // utiliser p.x, p.y
//!     }
//! }
//! ```
//!
//! ## Convention de grille
//!
//! La grille est indexée sous la forme `grille[y][x]` :
//! - `true`  → obstacle (case infranchissable)
//! - `false` → case libre (franchissable)
//!
//! ## Réglage des capacités
//!
//! Trois génériques constants pilotent l'utilisation mémoire :
//!
//! | Paramètre     | Rôle                             | Recommandation           |
//! |---------------|----------------------------------|--------------------------|
//! | `LARGEUR`     | Largeur de la grille (axe X)     | = largeur de votre carte |
//! | `HAUTEUR`     | Hauteur de la grille (axe Y)     | = hauteur de votre carte |
//! | `CAPACITE_MAX`| Capacité de l'open set et du chemin retourné | ≥ 2 × √(L × H) |
//!
//! Une grille 32×32 fonctionne bien avec `CAPACITE_MAX = 128`.
//! Une grille 64×64 nécessite `CAPACITE_MAX = 256` ou plus.

#![no_std]
#![deny(missing_docs)]
#![deny(unsafe_code)]

use heapless::Vec;

// ───────────────────────────────────────────────
//  Types publics
// ───────────────────────────────────────────────

/// Coordonnée entière 2D sur la grille.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Point {
    /// Position horizontale (indice de colonne).
    pub x: i32,
    /// Position verticale (indice de ligne).
    pub y: i32,
}

impl Point {
    /// Crée un nouveau [`Point`] à partir de ses coordonnées.
    #[inline]
    pub const fn nouveau(x: i32, y: i32) -> Self {
        Self { x, y }
    }
}

// ───────────────────────────────────────────────
//  Nœud interne
// ───────────────────────────────────────────────

#[derive(Clone, Copy)]
struct Noeud {
    point: Point,
    /// Coût réel depuis le départ jusqu'à ce nœud.
    cout_g: i32,
    /// Coût estimé total : cout_g + heuristique vers l'arrivée.
    cout_f: i32,
}

// ───────────────────────────────────────────────
//  Heuristique
// ───────────────────────────────────────────────

/// Distance de Manhattan : heuristique admissible pour les grilles à 4 directions.
///
/// Pour un déplacement diagonal, remplacer par la distance de Chebyshev.
#[inline]
fn heuristique(a: Point, b: Point) -> i32 {
    (a.x - b.x).abs() + (a.y - b.y).abs()
}

// ───────────────────────────────────────────────
//  Vérification de validité
// ───────────────────────────────────────────────

/// Retourne `true` si le point est dans les bornes et n'est pas un obstacle.
#[inline]
fn est_valide<const L: usize, const H: usize>(
    p: Point,
    grille: &[[bool; L]; H],
) -> bool {
    p.x >= 0
        && (p.x as usize) < L
        && p.y >= 0
        && (p.y as usize) < H
        && !grille[p.y as usize][p.x as usize]
}

// ───────────────────────────────────────────────
//  API publique
// ───────────────────────────────────────────────

/// Exécute l'algorithme A\* sur une grille booléenne d'obstacles.
///
/// # Paramètres de type
///
/// - `LARGEUR`      — nombre de colonnes de la grille
/// - `HAUTEUR`      — nombre de lignes de la grille
/// - `CAPACITE_MAX` — capacité maximale de l'open set **et** du chemin retourné.
///   Doit être suffisamment grand pour contenir le chemin le plus long attendu.
///
/// # Arguments
///
/// - `depart`  — case de départ (doit être libre et dans les bornes)
/// - `arrivee` — case d'arrivée (doit être libre et dans les bornes)
/// - `grille`  — tableau booléen 2D indexé en `grille[y][x]`.
///   `true` = obstacle, `false` = case franchissable.
///
/// # Valeur de retour
///
/// `Some(chemin)`: un [`heapless::Vec`] de [`Point`]s ordonné du départ (inclus)
/// jusqu'à l'arrivée (incluse), représentant le chemin optimal.
///
/// `None` : aucun chemin n'existe, un point de terminaison est un obstacle ou
/// hors bornes, ou le chemin dépasse la capacité `CAPACITE_MAX`.
///
/// # Exemple
///
/// ```rust
/// use embedded_astar::{trouver_chemin, Point};
///
/// let grille = [[false; 4]; 4]; // grille 4×4 vide
/// let chemin = trouver_chemin::<4, 4, 16>(
///     Point::nouveau(0, 0),
///     Point::nouveau(3, 3),
///     &grille,
/// )
/// .expect("un chemin doit exister sur une grille vide");
///
/// assert_eq!(*chemin.first().unwrap(), Point::nouveau(0, 0));
/// assert_eq!(*chemin.last().unwrap(),  Point::nouveau(3, 3));
/// ```
pub fn trouver_chemin<const LARGEUR: usize, const HAUTEUR: usize, const CAPACITE_MAX: usize>(
    depart: Point,
    arrivee: Point,
    grille: &[[bool; LARGEUR]; HAUTEUR],
) -> Option<Vec<Point, CAPACITE_MAX>> {
    // Rejeter immédiatement les points invalides ou bloqués.
    if !est_valide::<LARGEUR, HAUTEUR>(depart, grille)
        || !est_valide::<LARGEUR, HAUTEUR>(arrivee, grille)
    {
        return None;
    }

    // ── Structures de données ─────────────────────────────────────────────────

    // Open set : nœuds en attente d'évaluation.
    let mut ouvert: Vec<Noeud, CAPACITE_MAX> = Vec::new();

    // Ensemble fermé : cases déjà entièrement évaluées (évite les ré-expansions).
    let mut ferme: [[bool; LARGEUR]; HAUTEUR] = [[false; LARGEUR]; HAUTEUR];

    // Carte des parents : pour chaque case, quelle case l'a atteinte.
    // Utilisée pour reconstruire le chemin en fin d'algorithme.
    let mut parents: [[Option<Point>; LARGEUR]; HAUTEUR] = [[None; LARGEUR]; HAUTEUR];

    // ── Initialisation ────────────────────────────────────────────────────────

    let noeud_depart = Noeud {
        point: depart,
        cout_g: 0,
        cout_f: heuristique(depart, arrivee),
    };
    // L'open set est vide : le push ne peut pas échouer ici.
    let _ = ouvert.push(noeud_depart);

    // Directions : Haut, Droite, Bas, Gauche (4-directionnel).
    // Pour activer les diagonales, ajouter (±1, ±1) et utiliser la distance de Chebyshev.
    const DIRECTIONS: [(i32, i32); 4] = [(0, -1), (1, 0), (0, 1), (-1, 0)];

    // ── Boucle principale ─────────────────────────────────────────────────────

    while !ouvert.is_empty() {
        // Sélectionner le nœud au cout_f le plus faible via scan linéaire.
        // Adapté aux petits open sets typiques des grilles embarquées.
        // Pour CAPACITE_MAX > 256, envisager un tas min.
        let mut idx_meilleur = 0;
        for i in 1..ouvert.len() {
            if ouvert[i].cout_f < ouvert[idx_meilleur].cout_f {
                idx_meilleur = i;
            }
        }

        // Extraire le meilleur nœud en O(1) via swap_remove.
        let courant = ouvert.swap_remove(idx_meilleur);
        let pc = courant.point;

        // ── Arrivée atteinte ──────────────────────────────────────────────────

        if pc == arrivee {
            return reconstruire_chemin::<LARGEUR, HAUTEUR, CAPACITE_MAX>(
                arrivee,
                depart,
                &parents,
            );
        }

        // Marquer la case comme visitée.
        ferme[pc.y as usize][pc.x as usize] = true;

        // ── Expansion des voisins ─────────────────────────────────────────────

        for &(dx, dy) in &DIRECTIONS {
            let voisin = Point {
                x: pc.x + dx,
                y: pc.y + dy,
            };

            // Ignorer les cases hors bornes, les obstacles et les cases déjà évaluées.
            if !est_valide::<LARGEUR, HAUTEUR>(voisin, grille)
                || ferme[voisin.y as usize][voisin.x as usize]
            {
                continue;
            }

            let g_tentative = courant.cout_g + 1; // coût uniforme : 1 par déplacement

            // Chercher si ce voisin est déjà dans l'open set.
            let existant = ouvert.iter_mut().find(|n| n.point == voisin);

            match existant {
                Some(noeud) if g_tentative < noeud.cout_g => {
                    // Meilleur chemin trouvé vers un voisin déjà en file : mise à jour.
                    noeud.cout_g = g_tentative;
                    noeud.cout_f = g_tentative + heuristique(voisin, arrivee);
                    parents[voisin.y as usize][voisin.x as usize] = Some(pc);
                }
                None => {
                    // Première découverte de ce voisin.
                    parents[voisin.y as usize][voisin.x as usize] = Some(pc);
                    let noeud_voisin = Noeud {
                        point: voisin,
                        cout_g: g_tentative,
                        cout_f: g_tentative + heuristique(voisin, arrivee),
                    };
                    // Si l'open set est plein, ignorer ce voisin sans paniquer.
                    let _ = ouvert.push(noeud_voisin);
                }
                _ => { /* le nœud existant a déjà un cout_g égal ou meilleur */ }
            }
        }
    }

    None // Tous les nœuds accessibles ont été explorés sans atteindre l'arrivée.
}

// ───────────────────────────────────────────────
//  Reconstruction du chemin (interne)
// ───────────────────────────────────────────────

fn reconstruire_chemin<const L: usize, const H: usize, const CAP: usize>(
    arrivee: Point,
    depart: Point,
    parents: &[[Option<Point>; L]; H],
) -> Option<Vec<Point, CAP>> {
    let mut chemin: Vec<Point, CAP> = Vec::new();
    let mut courant = arrivee;

    // Remonter de l'arrivée vers le départ via la carte des parents.
    loop {
        if chemin.push(courant).is_err() {
            // Le chemin dépasse la capacité allouée : signaler l'échec proprement.
            return None;
        }
        if courant == depart {
            break;
        }
        match parents[courant.y as usize][courant.x as usize] {
            Some(p) => courant = p,
            None => return None, // Déconnexion : ne devrait pas survenir en usage normal.
        }
    }

    chemin.reverse();
    Some(chemin)
}

// ───────────────────────────────────────────────
//  Tests
// ───────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    type Grille4 = [[bool; 4]; 4];

    fn grille_vide() -> Grille4 {
        [[false; 4]; 4]
    }

    #[test]
    fn depart_egal_arrivee() {
        let grille = grille_vide();
        let p = Point::nouveau(1, 1);
        let chemin = trouver_chemin::<4, 4, 16>(p, p, &grille).unwrap();
        assert_eq!(chemin.len(), 1);
        assert_eq!(chemin[0], p);
    }

    #[test]
    fn deplacement_horizontal() {
        let grille = grille_vide();
        let chemin = trouver_chemin::<4, 4, 16>(
            Point::nouveau(0, 0),
            Point::nouveau(3, 0),
            &grille,
        )
        .unwrap();
        assert_eq!(chemin.first().copied(), Some(Point::nouveau(0, 0)));
        assert_eq!(chemin.last().copied(),  Some(Point::nouveau(3, 0)));
        assert_eq!(chemin.len(), 4); // (0,0)→(1,0)→(2,0)→(3,0)
    }

    #[test]
    fn deplacement_vertical() {
        let grille = grille_vide();
        let chemin = trouver_chemin::<4, 4, 16>(
            Point::nouveau(0, 0),
            Point::nouveau(0, 3),
            &grille,
        )
        .unwrap();
        assert_eq!(chemin.len(), 4);
    }

    #[test]
    fn chemin_diagonal() {
        let grille = grille_vide();
        let chemin = trouver_chemin::<4, 4, 16>(
            Point::nouveau(0, 0),
            Point::nouveau(3, 3),
            &grille,
        )
        .unwrap();
        assert_eq!(chemin.first().copied(), Some(Point::nouveau(0, 0)));
        assert_eq!(chemin.last().copied(),  Some(Point::nouveau(3, 3)));
        // Optimal Manhattan = 6 déplacements → 7 points
        assert_eq!(chemin.len(), 7);
    }

    #[test]
    fn mur_avec_passage() {
        // Grille :
        //  . . . .
        //  # # . #     # = obstacle
        //  . . . .
        //  . . . .
        // Passage en (x=2, y=1) ; le chemin doit y passer.
        let mut grille = grille_vide();
        grille[1][0] = true;
        grille[1][1] = true;
        grille[1][3] = true;

        let chemin = trouver_chemin::<4, 4, 16>(
            Point::nouveau(0, 0),
            Point::nouveau(0, 2),
            &grille,
        )
        .unwrap();
        assert_eq!(chemin.first().copied(), Some(Point::nouveau(0, 0)));
        assert_eq!(chemin.last().copied(),  Some(Point::nouveau(0, 2)));
    }

    #[test]
    fn aucun_chemin_mur_total() {
        let mut grille = grille_vide();
        // Ligne 1 entièrement bloquée aucun passage possible.
        grille[1][0] = true;
        grille[1][1] = true;
        grille[1][2] = true;
        grille[1][3] = true;

        let resultat = trouver_chemin::<4, 4, 16>(
            Point::nouveau(0, 0),
            Point::nouveau(0, 2),
            &grille,
        );
        assert!(resultat.is_none());
    }

    #[test]
    fn depart_obstacle() {
        let mut grille = grille_vide();
        grille[0][0] = true;
        let resultat = trouver_chemin::<4, 4, 16>(
            Point::nouveau(0, 0),
            Point::nouveau(3, 3),
            &grille,
        );
        assert!(resultat.is_none());
    }

    #[test]
    fn arrivee_obstacle() {
        let mut grille = grille_vide();
        grille[3][3] = true;
        let resultat = trouver_chemin::<4, 4, 16>(
            Point::nouveau(0, 0),
            Point::nouveau(3, 3),
            &grille,
        );
        assert!(resultat.is_none());
    }

    #[test]
    fn hors_bornes() {
        let grille = grille_vide();
        let resultat = trouver_chemin::<4, 4, 16>(
            Point::nouveau(0, 0),
            Point::nouveau(99, 99),
            &grille,
        );
        assert!(resultat.is_none());
    }
}