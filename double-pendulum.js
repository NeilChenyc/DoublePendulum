class DoublePendulum {
    constructor() {
        // Physics parameters
        this.g = 9.81;
        this.L1 = 120;
        this.L2 = 120;
        this.m1 = 1;
        this.m2 = 1;

        // Initial state
        this.theta1 = 120 * Math.PI / 180;
        this.theta2 = -10 * Math.PI / 180;
        this.omega1 = 0;
        this.omega2 = 0;

        // State variables for integration
        this.state = [this.theta1, this.omega1, this.theta2, this.omega2];

        // Trajectory tracking
        this.trace = [];
        this.maxTracePoints = 3000;

        // Animation state
        this.isRunning = true;
        this.lastTime = 0;
        this.dt = 1/240; // Fixed time step
        this.maxStepPerFrame = 10; // Prevent large jumps when tab is hidden

        // Canvas setup
        this.canvas = document.getElementById('canvas');
        this.ctx = this.canvas.getContext('2d');
        this.centerX = this.canvas.width / 2;
        this.centerY = this.canvas.height / 3;

        // UI elements
        this.statusElement = document.getElementById('status');
        this.tracePointsElement = document.getElementById('tracePoints');
        this.anglesElement = document.getElementById('angles');

        this.setupEventListeners();
        this.animate();
    }

    // Calculate derivatives for the double pendulum ODE
    derivatives(state, dt) {
        const [theta1, omega1, theta2, omega2] = state;

        const d1 = this.L1 * this.L1 * this.m2;
        const d2 = this.L2 * this.L2 * (this.m1 + this.m2);
        const d3 = this.L1 * this.L2 * this.m2 * Math.cos(theta1 - theta2);
        const d4 = this.L1 * this.L1 * this.L2 * (this.m1 + this.m2);

        const num1 = -this.g * (2 * this.m1 + this.m2) * Math.sin(theta1);
        const num2 = -this.m2 * this.g * Math.sin(theta1 - 2 * theta2);
        const num3 = -2 * Math.sin(theta1 - theta2) * this.m2 * (omega2 * omega2 * this.L2 + omega1 * omega1 * this.L1 * Math.cos(theta1 - theta2));
        const den1 = this.L1 * (2 * this.m1 + this.m2 - this.m2 * Math.cos(2 * theta1 - 2 * theta2));

        const num4 = 2 * Math.sin(theta1 - theta2);
        const num5 = omega1 * omega1 * this.L1 * (this.m1 + this.m2);
        const num6 = this.g * (this.m1 + this.m2) * Math.cos(theta1);
        const num7 = omega2 * omega2 * this.L2 * this.m2 * Math.cos(theta1 - theta2);
        const den2 = this.L2 * (2 * this.m1 + this.m2 - this.m2 * Math.cos(2 * theta1 - 2 * theta2));

        const alpha1 = (num1 + num2 + num3) / den1;
        const alpha2 = (num4 * (num5 + num6 + num7)) / den2;

        // Add slight damping for numerical stability
        const damping = 0.9999;
        const dtheta1 = omega1 * damping;
        const dtheta2 = omega2 * damping;

        return [dtheta1, alpha1, dtheta2, alpha2];
    }

    // Runge-Kutta 4 integration
    rk4(state, dt) {
        const k1 = this.derivatives(state, dt);
        const k2 = this.derivatives(state.map((s, i) => s + k1[i] * dt / 2), dt);
        const k3 = this.derivatives(state.map((s, i) => s + k2[i] * dt / 2), dt);
        const k4 = this.derivatives(state.map((s, i) => s + k3[i] * dt), dt);

        return state.map((s, i) => s + (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) * dt / 6);
    }

    // Update the pendulum state
    update(deltaTime) {
        if (!this.isRunning) return;

        // Handle large time steps (e.g., when tab is hidden)
        let remainingTime = deltaTime;
        let steps = 0;

        while (remainingTime > 0 && steps < this.maxStepPerFrame) {
            const stepTime = Math.min(remainingTime, this.dt);
            this.state = this.rk4(this.state, stepTime);
            remainingTime -= stepTime;
            steps++;
        }

        // Update angles and velocities
        [this.theta1, this.omega1, this.theta2, this.omega2] = this.state;

        // Calculate positions
        this.x1 = this.centerX + this.L1 * Math.sin(this.theta1);
        this.y1 = this.centerY + this.L1 * Math.cos(this.theta1);
        this.x2 = this.x1 + this.L2 * Math.sin(this.theta2);
        this.y2 = this.y1 + this.L2 * Math.cos(this.theta2);

        // Add to trace
        this.trace.push({ x: this.x2, y: this.y2 });

        // Limit trace length
        if (this.trace.length > this.maxTracePoints) {
            this.trace.shift();
        }
    }

    // Render the pendulum and trajectory
    render() {
        // Clear canvas
        this.ctx.fillStyle = '#0a0a0a';
        this.ctx.fillRect(0, 0, this.canvas.width, this.canvas.height);

        // Draw trace
        if (this.trace.length > 1) {
            this.ctx.beginPath();
            this.ctx.strokeStyle = 'rgba(100, 200, 255, 0.5)';
            this.ctx.lineWidth = 1;

            this.trace.forEach((point, index) => {
                if (index === 0) {
                    this.ctx.moveTo(point.x, point.y);
                } else {
                    this.ctx.lineTo(point.x, point.y);
                }
            });

            this.ctx.stroke();
        }

        // Draw pendulum
        const scale = 2; // Mass visualization scale

        // Pivot point
        this.ctx.beginPath();
        this.ctx.arc(this.centerX, this.centerY, 5, 0, Math.PI * 2);
        this.ctx.fillStyle = '#fff';
        this.ctx.fill();

        // First摆杆
        this.ctx.beginPath();
        this.ctx.moveTo(this.centerX, this.centerY);
        this.ctx.lineTo(this.x1, this.y1);
        this.ctx.strokeStyle = '#666';
        this.ctx.lineWidth = 2;
        this.ctx.stroke();

        // First摆锤
        this.ctx.beginPath();
        this.ctx.arc(this.x1, this.y1, this.m1 * scale, 0, Math.PI * 2);
        this.ctx.fillStyle = '#ff6666';
        this.ctx.fill();
        this.ctx.strokeStyle = '#cc4444';
        this.ctx.lineWidth = 1;
        this.ctx.stroke();

        // Second摆杆
        this.ctx.beginPath();
        this.ctx.moveTo(this.x1, this.y1);
        this.ctx.lineTo(this.x2, this.y2);
        this.ctx.strokeStyle = '#666';
        this.ctx.lineWidth = 2;
        this.ctx.stroke();

        // Second摆锤 (末端)
        this.ctx.beginPath();
        this.ctx.arc(this.x2, this.y2, this.m2 * scale, 0, Math.PI * 2);
        this.ctx.fillStyle = '#66ff66';
        this.ctx.fill();
        this.ctx.strokeStyle = '#44cc44';
        this.ctx.lineWidth = 1;
        this.ctx.stroke();

        // Update UI info
        this.updateUI();
    }

    // Update UI information
    updateUI() {
        this.statusElement.textContent = `Status: ${this.isRunning ? 'Running' : 'Paused'}`;
        this.tracePointsElement.textContent = `Trace points: ${this.trace.length}`;

        const theta1Deg = (this.theta1 * 180 / Math.PI).toFixed(1);
        const theta2Deg = (this.theta2 * 180 / Math.PI).toFixed(1);
        this.anglesElement.textContent = `θ1: ${theta1Deg}°, θ2: ${theta2Deg}°`;
    }

    // Reset to initial state
    reset() {
        this.theta1 = 120 * Math.PI / 180;
        this.theta2 = -10 * Math.PI / 180;
        this.omega1 = 0;
        this.omega2 = 0;
        this.state = [this.theta1, this.omega1, this.theta2, this.omega2];
        this.trace = [];
        this.lastTime = 0;
    }

    // Clear trace
    clearTrace() {
        this.trace = [];
    }

    // Setup event listeners
    setupEventListeners() {
        document.addEventListener('keydown', (e) => {
            switch(e.code) {
                case 'Space':
                    e.preventDefault();
                    this.isRunning = !this.isRunning;
                    this.lastTime = 0; // Reset time to prevent large delta
                    break;
                case 'KeyR':
                    this.reset();
                    break;
                case 'KeyC':
                    this.clearTrace();
                    break;
            }
        });
    }

    // Animation loop
    animate(currentTime = 0) {
        if (this.lastTime === 0) {
            this.lastTime = currentTime;
        }

        const deltaTime = (currentTime - this.lastTime) / 1000; // Convert to seconds
        this.update(deltaTime);
        this.render();

        this.lastTime = currentTime;
        requestAnimationFrame((time) => this.animate(time));
    }
}

// Start the simulation
window.addEventListener('load', () => {
    new DoublePendulum();
});