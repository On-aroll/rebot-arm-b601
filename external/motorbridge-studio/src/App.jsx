import React from 'react';
import { BrandWatermark } from './components/BrandWatermark';
import { HeaderBar } from './components/HeaderBar';
import { QuickMenu } from './components/QuickMenu';
import { ConnectionPanel } from './components/ConnectionPanel';
import { ScanWorkspace } from './components/ScanWorkspace';
import { MotorSection } from './components/MotorSection';
import { StateLogsPanel } from './components/StateLogsPanel';
import { RobotArmPage } from './components/RobotArmPage';
import { SimuPage } from './third_page';
import { ConsoleWorkspace } from './components/ConsoleWorkspace';
import { HelpCenterModal } from './components/HelpCenterModal';
import { ConfirmDialog } from './components/ConfirmDialog';
import { useMotorStudio } from './hooks/useMotorStudio';
import { MotorStudioProvider } from './hooks/useMotorStudioContext';
import { useI18n } from './i18n';
import appPackage from '../package.json';

export default function App() {
  const { t } = useI18n();
  const studio = useMotorStudio();
  // /writing 是改名前的旧入口，继续接受以免书签和文档失效。
  const [page, setPage] = React.useState(/\/(console|writing)$/.test(window.location.pathname) ? 'console' : 'general');
  const [helpOpen, setHelpOpen] = React.useState(false);
  const version = `v${appPackage.version}`;

  const pathParts = String(window.location.pathname || '/').replace(import.meta.env.BASE_URL, '/')
    .split('/')
    .filter(Boolean);
  const leading = pathParts[0];
  const isLocale = leading === 'en' || leading === 'zh' || leading === 'es';
  const route = isLocale ? pathParts[1] || '' : pathParts[0] || '';
  const isSimuRoute = route === 'simu';

  if (isSimuRoute) {
    return (
      <MotorStudioProvider value={studio}>
        <SimuPage />
      </MotorStudioProvider>
    );
  }

  return (
    <MotorStudioProvider value={studio}>
      <div className={`app shell ${page === 'console' ? 'consoleApp' : ''}`}>
        <BrandWatermark />
        {/* 控制台自带网关连接与状态徽章，这里再显示一个"未连接"只会误导。 */}
        <HeaderBar simulationWorkspace={page === 'console'} hideGatewayBadge={page === 'console'} />

        {studio.workspace.menuOpen && <QuickMenu />}

        <section className="card glass">
          <div className="row toolbar compactToolbar">
            <button className={page === 'general' ? 'primary' : ''} onClick={() => setPage('general')}>
              {t('page_general')}
            </button>
            <button className={page === 'robot_arm' ? 'primary' : ''} onClick={() => setPage('robot_arm')}>
              {t('page_robot_arm')}
            </button>
            <button className={page === 'console' ? 'primary' : ''} onClick={() => setPage('console')}>
              {t('page_console')}
            </button>
            <button className="ghostBtn" onClick={() => setHelpOpen(true)}>
              {t('help_show')}
            </button>
            <button
              className="ghostBtn"
              style={{ marginLeft: 'auto' }}
              onClick={() => studio.scan?.clearDevices()}
              title={t('confirm_clear_all')}
            >
              {t('clear_all_motors')}
            </button>
          </div>
        </section>

        <HelpCenterModal open={helpOpen} page={page} onClose={() => setHelpOpen(false)} />
        <ConfirmDialog
          open={Boolean(studio.scan?.confirmDialog?.open)}
          title={studio.scan?.confirmDialog?.title}
          message={studio.scan?.confirmDialog?.message}
          danger={Boolean(studio.scan?.confirmDialog?.danger)}
          onCancel={() => studio.scan?.closeConfirmDialog(false)}
          onConfirm={() => studio.scan?.closeConfirmDialog(true)}
        />

        {page !== 'console' && <ConnectionPanel />}

        {page === 'console' ? <ConsoleWorkspace /> : page === 'general' ? (
          <>
            <ScanWorkspace />
            <MotorSection />
          </>
        ) : (
          <RobotArmPage />
        )}

        {page !== 'console' && <StateLogsPanel />}
        <div className="appFooterMeta">{version}</div>
      </div>
    </MotorStudioProvider>
  );
}
