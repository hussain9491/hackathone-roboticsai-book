import React, { useEffect } from 'react';
import { useLocation } from '@docusaurus/router';

const SidebarEnhancer: React.FC = () => {
  const location = useLocation();

  useEffect(() => {
    // Add chapter numbers to sidebar items after DOM updates
    const addChapterNumbers = () => {
      const menuLinks = document.querySelectorAll('.menu__list .menu__link');

      menuLinks.forEach((link, index) => {
        const linkElement = link as HTMLElement;
        const parentCategory = linkElement.closest('.menu__list-item-collapsible');

        if (parentCategory) {
          // This is a category item, add numbering if it's a chapter
          const label = linkElement.textContent || '';
          const chapterMatch = label.match(/^(\d+)\.\s+(.+)$/);
          if (chapterMatch) {
            const chapterNumber = chapterMatch[1].padStart(2, '0');
            linkElement.setAttribute('data-chapter-number', chapterNumber);
          }
        } else {
          // This is a regular menu item - we could add section numbers if needed
          // For now, we'll just ensure it doesn't have the chapter number attribute
          if (linkElement.hasAttribute('data-chapter-number')) {
            linkElement.removeAttribute('data-chapter-number');
          }
        }
      });
    };

    // Run immediately and after each route change
    addChapterNumbers();

    // Set up a simple observer to handle dynamic updates
    const observer = new MutationObserver(() => {
      addChapterNumbers();
    });

    observer.observe(document.body, {
      childList: true,
      subtree: true,
    });

    return () => {
      observer.disconnect();
    };
  }, [location.pathname]);

  return null;
};

export default SidebarEnhancer;