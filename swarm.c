
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

#define NUM_DRONES     15
#define NUM_TARGETS    3
#define MAX_SPEED      3.0
#define NEIGHBOR_RADIUS 50.0
#define TARGET_ATTRACT 0.02
#define SEARCH_RADIUS   20.0
#define SEARCH_DURATION 30      // timesteps spent scanning

typedef struct {
    double x, y, z;    // Position
    double vx, vy, vz; // Velocity
    bool is_searching;
    int search_timer;
} Drone;

typedef struct {
    double x, y, z;    // Target position
    bool searched;
} Target;

Drone swarm[NUM_DRONES];
Target zones[NUM_TARGETS];

// Clamp speed for realism
void limit_speed(double* vx, double* vy, double* vz) {
    double speed = sqrt((*vx)*(*vx) + (*vy)*(*vy) + (*vz)*(*vz));
    if (speed > MAX_SPEED) {
        *vx = (*vx / speed) * MAX_SPEED;
        *vy = (*vy / speed) * MAX_SPEED;
        *vz = (*vz / speed) * MAX_SPEED;
    }
}

// Find nearest unsearched target zone
int nearest_target(Drone* drone) {
    int closest = -1;
    double min_dist = 1e6;
    for (int i = 0; i < NUM_TARGETS; i++) {
        if (zones[i].searched) continue;
        double dx = zones[i].x - drone->x;
        double dy = zones[i].y - drone->y;
        double dz = zones[i].z - drone->z;
        double dist = sqrt(dx*dx + dy*dy + dz*dz);
        if (dist < min_dist) {
            min_dist = dist;
            closest = i;
        }
    }
    return closest;
}

// Drone update with search behavior
void update_drone(int id) {
    Drone* self = &swarm[id];

    // If searching, hold position and count down
    if (self->is_searching) {
        self->vx = self->vy = self->vz = 0;
        self->search_timer--;
        if (self->search_timer <= 0) {
            self->is_searching = false;
        }
        return;
    }

    double coh_x = 0, coh_y = 0, coh_z = 0;
    double sep_x = 0, sep_y = 0, sep_z = 0;
    double ali_x = 0, ali_y = 0, ali_z = 0;
    int neighbors = 0;

    for (int i = 0; i < NUM_DRONES; i++) {
        if (i == id) continue;
        Drone* other = &swarm[i];
        double dx = other->x - self->x;
        double dy = other->y - self->y;
        double dz = other->z - self->z;
        double dist = sqrt(dx*dx + dy*dy + dz*dz);

        if (dist < NEIGHBOR_RADIUS) {
            coh_x += other->x;
            coh_y += other->y;
            coh_z += other->z;

            sep_x -= dx / dist;
            sep_y -= dy / dist;
            sep_z -= dz / dist;

            ali_x += other->vx;
            ali_y += other->vy;
            ali_z += other->vz;

            neighbors++;
        }
    }

    if (neighbors > 0) {
        coh_x = (coh_x / neighbors - self->x) * 0.01;
        coh_y = (coh_y / neighbors - self->y) * 0.01;
        coh_z = (coh_z / neighbors - self->z) * 0.01;

        sep_x *= 0.05;
        sep_y *= 0.05;
        sep_z *= 0.05;

        ali_x = (ali_x / neighbors - self->vx) * 0.05;
        ali_y = (ali_y / neighbors - self->vy) * 0.05;
        ali_z = (ali_z / neighbors - self->vz) * 0.05;

        self->vx += coh_x + sep_x + ali_x;
        self->vy += coh_y + sep_y + ali_y;
        self->vz += coh_z + sep_z + ali_z;
    }

    // Move toward nearest unsearched target zone
    int target_idx = nearest_target(self);
    if (target_idx >= 0) {
        Target* target = &zones[target_idx];
        double dx = target->x - self->x;
        double dy = target->y - self->y;
        double dz = target->z - self->z;
        double dist = sqrt(dx*dx + dy*dy + dz*dz);

        if (dist < SEARCH_RADIUS) {
            // Begin search
            self->is_searching = true;
            self->search_timer = SEARCH_DURATION;
            target->searched = true;
        } else {
            // Attract toward target
            self->vx += TARGET_ATTRACT * dx / dist;
            self->vy += TARGET_ATTRACT * dy / dist;
            self->vz += TARGET_ATTRACT * dz / dist;
        }
    }

    limit_speed(&self->vx, &self->vy, &self->vz);
    self->x += self->vx;
    self->y += self->vy;
    self->z += self->vz;
}

// Main simulation
int main() {
    for (int i = 0; i < NUM_DRONES; i++) {
        swarm[i].x = rand() % 300;
        swarm[i].y = rand() % 300;
        swarm[i].z = rand() % 50 + 20;
        swarm[i].vx = swarm[i].vy = swarm[i].vz = 0;
        swarm[i].is_searching = false;
        swarm[i].search_timer = 0;
    }

    zones[0] = (Target){ 100, 250, 30, false };
    zones[1] = (Target){ 280, 100, 45, false };
    zones[2] = (Target){ 150, 180, 40, false };

    for (int step = 0; step < 600; step++) {
        for (int i = 0; i < NUM_DRONES; i++) {
            update_drone(i);
            printf("D%02d Pos=(%.1f, %.1f, %.1f) %s\n",
                   i, swarm[i].x, swarm[i].y, swarm[i].z,
                   swarm[i].is_searching ? "[Searching]" : "");
        }

        // Check completion
        bool all_done = true;
        for (int t = 0; t < NUM_TARGETS; t++) {
            if (!zones[t].searched) {
                all_done = false;
                break;
            }
        }
        if (all_done) {
            printf(" All targets searched. Mission complete!\n");
            break;
        }

        printf("Step %d done.\n\n", step);
    }

    return 0;
}

