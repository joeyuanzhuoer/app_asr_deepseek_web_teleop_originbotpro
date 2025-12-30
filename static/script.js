// Web Teleop JavaScript for Turtlesim Control

let isPressing = false;
let currentCommand = null;

// Initialize on page load
document.addEventListener('DOMContentLoaded', function() {
    checkStatus();
    setupEventListeners();
    
    // Check status every 2 seconds
    setInterval(checkStatus, 2000);
});

// Check ROS2 status
function checkStatus() {
    fetch('/status')
        .then(response => response.json())
        .then(data => {
            const statusEl = document.getElementById('ros2-status');
            const linearEl = document.getElementById('linear-speed');
            const angularEl = document.getElementById('angular-speed');
            
            if (data.ros2_initialized) {
                statusEl.textContent = 'Connected';
                statusEl.style.color = '#4CAF50';
            } else {
                statusEl.textContent = 'Disconnected';
                statusEl.style.color = '#f44336';
            }
            
            linearEl.textContent = data.linear_speed.toFixed(2) + ' m/s';
            angularEl.textContent = data.angular_speed.toFixed(2) + ' rad/s';
        })
        .catch(error => {
            console.error('Error checking status:', error);
            document.getElementById('ros2-status').textContent = 'Error';
            document.getElementById('ros2-status').style.color = '#f44336';
        });
}

// Setup event listeners
function setupEventListeners() {
    // Button clicks
    document.querySelectorAll('.control-btn, .speed-btn').forEach(button => {
        button.addEventListener('mousedown', handleButtonPress);
        button.addEventListener('mouseup', handleButtonRelease);
        button.addEventListener('mouseleave', handleButtonRelease);
        button.addEventListener('touchstart', handleButtonPress, { passive: true });
        button.addEventListener('touchend', handleButtonRelease, { passive: true });
        button.addEventListener('touchcancel', handleButtonRelease, { passive: true });
    });
    
    // Keyboard controls (matching teleop_twist_keyboard layout)
    document.addEventListener('keydown', handleKeyDown);
    document.addEventListener('keyup', handleKeyUp);
    
    // Prevent default behavior for space key
    document.addEventListener('keydown', function(e) {
        if (e.key === ' ' || e.key === 'k') {
            e.preventDefault();
        }
    });
}

// Handle button press
function handleButtonPress(event) {
    event.preventDefault();
    const button = event.currentTarget;
    const command = button.getAttribute('data-command');
    
    if (command && !isPressing) {
        isPressing = true;
        currentCommand = command;
        button.classList.add('pressed');
        sendCommand(command);
        
        // For movement commands, keep sending while pressed
        if (isMovementCommand(command)) {
            startContinuousCommand(command);
        }
    }
}

// Handle button release
function handleButtonRelease(event) {
    event.preventDefault();
    const button = event.currentTarget;
    button.classList.remove('pressed');
    
    if (isPressing) {
        isPressing = false;
        stopContinuousCommand();
        
        // Send stop command for movement buttons
        if (isMovementCommand(currentCommand)) {
            sendCommand('stop');
        }
        
        currentCommand = null;
    }
}

// Handle keyboard down
function handleKeyDown(event) {
    if (isPressing) return;
    
    const key = event.key.toLowerCase();
    const keyMap = {
        'u': 'forward_left',
        'i': 'forward',
        'o': 'forward_right',
        'j': 'left',
        'k': 'stop',
        'l': 'right',
        'm': 'backward_left',
        ',': 'backward',
        '.': 'backward_right',
        'q': 'increase_speed',
        'z': 'decrease_speed',
        'w': 'increase_linear',
        'x': 'decrease_linear',
        'e': 'increase_angular',
        'c': 'decrease_angular',
        ' ': 'stop'
    };
    
    const command = keyMap[key];
    if (command) {
        event.preventDefault();
        isPressing = true;
        currentCommand = command;
        
        // Highlight corresponding button
        const button = document.querySelector(`[data-command="${command}"]`);
        if (button) {
            button.classList.add('pressed');
        }
        
        sendCommand(command);
        
        // For movement commands, keep sending while pressed
        if (isMovementCommand(command)) {
            startContinuousCommand(command);
        }
    }
}

// Handle keyboard up
function handleKeyUp(event) {
    const key = event.key.toLowerCase();
    const movementKeys = ['u', 'i', 'o', 'j', 'l', 'm', ',', '.'];
    
    if (movementKeys.includes(key) && isPressing) {
        event.preventDefault();
        isPressing = false;
        stopContinuousCommand();
        sendCommand('stop');
        
        // Remove highlight
        const keyMap = {
            'u': 'forward_left',
            'i': 'forward',
            'o': 'forward_right',
            'j': 'left',
            'l': 'right',
            'm': 'backward_left',
            ',': 'backward',
            '.': 'backward_right'
        };
        const command = keyMap[key];
        if (command) {
            const button = document.querySelector(`[data-command="${command}"]`);
            if (button) {
                button.classList.remove('pressed');
            }
        }
        
        currentCommand = null;
    }
}

// Check if command is a movement command
function isMovementCommand(command) {
    const movementCommands = [
        'forward', 'backward', 'left', 'right',
        'forward_left', 'forward_right',
        'backward_left', 'backward_right'
    ];
    return movementCommands.includes(command);
}

// Continuous command interval
let commandInterval = null;

// Start continuous command sending
function startContinuousCommand(command) {
    if (commandInterval) {
        clearInterval(commandInterval);
    }
    
    // Send command every 100ms while button is pressed
    commandInterval = setInterval(() => {
        if (isPressing && currentCommand === command) {
            sendCommand(command);
        } else {
            stopContinuousCommand();
        }
    }, 100);
}

// Stop continuous command sending
function stopContinuousCommand() {
    if (commandInterval) {
        clearInterval(commandInterval);
        commandInterval = null;
    }
}

// Send command to server
function sendCommand(command) {
    fetch('/cmd', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({ command: command })
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            // Update speed displays if they changed
            if (data.linear_speed !== undefined) {
                document.getElementById('linear-speed').textContent = 
                    data.linear_speed.toFixed(2) + ' m/s';
            }
            if (data.angular_speed !== undefined) {
                document.getElementById('angular-speed').textContent = 
                    data.angular_speed.toFixed(2) + ' rad/s';
            }
            
            // Show message if provided
            if (data.message) {
                showMessage(data.message, 'success');
            }
        } else {
            showMessage('Error: ' + (data.error || 'Unknown error'), 'error');
        }
    })
    .catch(error => {
        console.error('Error sending command:', error);
        showMessage('Error sending command: ' + error.message, 'error');
    });
}

// Show message
function showMessage(text, type) {
    const messageEl = document.getElementById('message');
    messageEl.textContent = text;
    messageEl.className = 'message ' + type;
    
    // Hide after 3 seconds
    setTimeout(() => {
        messageEl.className = 'message';
    }, 3000);
}

