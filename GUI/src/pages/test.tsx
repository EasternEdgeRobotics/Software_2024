import React, { useState } from 'react';
import { Sidebar, Menu, MenuItem } from 'react-pro-sidebar';
import { Home, Brightness4, SwapHoriz } from '@mui/icons-material';
import '../styles/index.css';

function App() {
    const [theme, setTheme] = useState('dark');
    const [position, setPosition] = useState('left');

    return (
        <Sidebar >
            <Menu>
                <MenuItem icon={<Home />}>Home</MenuItem>
                {/* More menu items... */}
            </Menu>
            <button onClick={() => setTheme(theme === 'dark' ? 'light' : 'dark')}>
                Toggle Theme <Brightness4 />
            </button>
            <button onClick={() => setPosition(position === 'left' ? 'right' : 'left')}>
                Toggle Position <SwapHoriz />
            </button>
        </Sidebar>
    );
}

export default App;