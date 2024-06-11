import Welcome from "../components/Welcome";
import DefaultDashBoard from "../layout/DefaultDashBoard";
import DefaultHome from "../layout/DefaultHome";
import DoorLight from "../pages/DoorLight";
import Fan from "../pages/Fan";
import Login from "../pages/Login";
import Sensor from "../pages/Sensor";
import Overview from "../pages/Overview";

export const routes = [
    {
        path: '/',
        element: <DefaultHome></DefaultHome>,
        children: [
            {
                index: true,
                element: <Welcome></Welcome>
            },
            {
                path: 'login',
                element: <Login></Login>
            }
        ]
    },
    {
        path: 'dashboard',
        element: <DefaultDashBoard></DefaultDashBoard>,
        children: [
            {
                path: 'overview',
                element: <Overview></Overview>
            },
            {
                path: 'door-light',
                element: <DoorLight></DoorLight>
            },
            {
                path: 'fan',
                element: <Fan></Fan>
            },
            {
                path: 'sensor',
                element: <Sensor></Sensor>
            }
        ]
    }
];