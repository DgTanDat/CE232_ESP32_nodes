import './Overview.scss';

function Overview() {
    return (
        <>
            <h3>LIST OF DEVICES</h3>
            <div className='deviceList'>
                <button className='btn btn-info add-new'>
                    <i class="fa-solid fa-plus"></i>
                    Add new
                </button>

                <table>
                    <thead>
                        <tr>
                            <th>ID</th>
                            <th>Name</th>
                            <th>Device</th>
                            <th>Status</th>
                            <th>Last Connection</th>
                            <th>Actions</th>
                        </tr>
                    </thead>

                    <tbody>
                        <tr>
                            <td>1</td>
                            <td>Light_1</td>
                            <td>Light</td>
                            <td>Disconnected</td>
                            <td>10:25:59 PM, 23/04/2024</td>
                            <td className='icon'>
                                <div>
                                    <i class="fa-solid fa-pencil"></i>
                                    <i class="fa-solid fa-trash"></i>
                                </div>
                            </td>
                        </tr>

                        <tr>
                            <td>2</td>
                            <td>Fan_1</td>
                            <td>Fan</td>
                            <td>Connected</td>
                            <td>09:17:50 AM, 24/04/2024</td>
                            <td className='icon'>
                                <div>
                                    <i class="fa-solid fa-pencil" ></i>
                                    <i class="fa-solid fa-trash"></i>
                                </div>
                            </td>
                        </tr>
                    </tbody>
                </table>
            </div>
        </>

    );
}

export default Overview;