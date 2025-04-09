const logDiv = document.getElementById("log");
const stepBtn = document.getElementById("step");

function log(text) {
  logDiv.innerText += text + "\n";
  logDiv.scrollTop = logDiv.scrollHeight;
}

class RigidBody {
  constructor(x, y, mass) {
    this.position = { x, y };
    this.velocity = { x: 0, y: 0 };
    this.force = { x: 0, y: 0 };
    this.mass = mass;
    this.invMass = mass > 0 ? 1 / mass : 0;
  }

  applyImpulse(impulse) {
    this.velocity.x += impulse.x * this.invMass;
    this.velocity.y += impulse.y * this.invMass;
  }

  integrate(dt) {
    this.velocity.x += this.force.x * this.invMass * dt;
    this.velocity.y += this.force.y * this.invMass * dt;
    this.position.x += this.velocity.x * dt;
    this.position.y += this.velocity.y * dt;
  }

  resetForce() {
    this.force = { x: 0, y: 0 };
  }
}

class ContactConstraint {
  constructor(bodyA, bodyB, normal, penetration) {
    this.bodyA = bodyA;
    this.bodyB = bodyB;
    this.normal = normal;
    this.penetration = penetration;
    this.lambdaAccum = 0;
  }

  solve() {
    const relVel = {
      x: this.bodyB.velocity.x - this.bodyA.velocity.x,
      y: this.bodyB.velocity.y - this.bodyA.velocity.y,
    };
    const velAlongNormal = relVel.x * this.normal.x + relVel.y * this.normal.y;

    const beta = 0.2;
    const baumgarte = beta * this.penetration;

    const effMass = this.bodyA.invMass + this.bodyB.invMass;
    if (effMass === 0) return;

    let lambda = -(velAlongNormal + baumgarte) / effMass;

    const lambdaOld = this.lambdaAccum;
    this.lambdaAccum = Math.max(this.lambdaAccum + lambda, 0);
    lambda = this.lambdaAccum - lambdaOld;

    const impulse = {
      x: this.normal.x * lambda,
      y: this.normal.y * lambda,
    };
    this.bodyA.applyImpulse({ x: -impulse.x, y: -impulse.y });
    this.bodyB.applyImpulse(impulse);

    log(`[PGS] λ=${lambda.toFixed(4)} | Impulse=(${impulse.x.toFixed(4)}, ${impulse.y.toFixed(4)})`);
  }
}

const A = new RigidBody(0, 0, 1);
const B = new RigidBody(0, -1, 1);
const contact = new ContactConstraint(A, B, { x: 0, y: 1 }, 0.1);
const bodies = [A, B];
const constraints = [contact];

function simulateStep() {
  simulate(bodies, constraints, 1 / 60, 8);
  log(`A.pos=(${A.position.x.toFixed(3)}, ${A.position.y.toFixed(3)})`);
  log(`B.pos=(${B.position.x.toFixed(3)}, ${B.position.y.toFixed(3)})\n`);
}

function simulate(bodies, constraints, dt, iterations = 10) {
  for (let body of bodies) {
    body.integrate(dt);
  }

  for (let i = 0; i < iterations; i++) {
    for (let constraint of constraints) {
      constraint.solve();
    }
  }

  for (let body of bodies) {
    body.resetForce();
  }
}

stepBtn?.addEventListener("click", simulateStep);
