// Tabs: one panel visible at a time, driven by the URL hash so links and the back button work.
(function () {
  var panels = Array.prototype.slice.call(document.querySelectorAll('.tab-panel'));
  var links = document.querySelectorAll('.tabs a');
  var ids = panels.map(function (p) { return p.id; });

  function show(hash, scroll) {
    var id = (hash || '').replace('#', '');
    var target = id ? document.getElementById(id) : null;
    var panel = target && target.classList.contains('tab-panel') ? target
              : target ? target.closest('.tab-panel') : null;
    if (!panel) panel = document.getElementById(ids[0]);

    panels.forEach(function (p) { p.hidden = p !== panel; });
    links.forEach(function (a) {
      if (a.getAttribute('href') === '#' + panel.id) a.setAttribute('aria-current', 'page');
      else a.removeAttribute('aria-current');
    });
    var tabLink = document.querySelector('.tabs a[href="#' + panel.id + '"]');
    document.title = (panel.id === 'overview' || !tabLink ? '' : tabLink.textContent + ' | ') + 'Helmet Impact Monitor | Oliver Comfort';

    if (scroll === false) return;
    if (target && target !== panel) setTimeout(function () { target.scrollIntoView({ behavior: 'instant' }); }, 60);
    else window.scrollTo(0, 0);
  }

  window.addEventListener('hashchange', function () { show(location.hash); });
  show(location.hash, !!location.hash);
})();

// Pause looping videos that are off screen.
if ('IntersectionObserver' in window) {
  var vo = new IntersectionObserver(function (entries) {
    entries.forEach(function (e) { if (e.isIntersecting) e.target.play().catch(function () {}); else e.target.pause(); });
  });
  document.querySelectorAll('video').forEach(function (v) { vo.observe(v); });
}
if (window.matchMedia('(prefers-reduced-motion: reduce)').matches) {
  document.querySelectorAll('video').forEach(function (v) { v.removeAttribute('autoplay'); v.pause(); v.controls = true; });
}

// Design log: highlight the section in view in its table of contents.
(function () {
  var links = document.querySelectorAll('.toc a');
  if (!links.length || !('IntersectionObserver' in window)) return;
  var byId = {};
  links.forEach(function (a) { byId[a.getAttribute('href').slice(1)] = a; });
  var observer = new IntersectionObserver(function (entries) {
    entries.forEach(function (entry) {
      if (entry.isIntersecting) {
        links.forEach(function (a) { a.classList.remove('active'); });
        var link = byId[entry.target.id];
        if (link) link.classList.add('active');
      }
    });
  }, { rootMargin: '-20% 0px -70% 0px' });
  document.querySelectorAll('.log section[id]').forEach(function (s) { observer.observe(s); });
})();

// Click a photo to see it full size.
(function () {
  if (!window.HTMLDialogElement) return;
  var box = document.createElement('dialog');
  box.className = 'lightbox';
  box.innerHTML = '<button type="button">Close</button><img alt=""><p></p>';
  document.body.appendChild(box);
  var big = box.querySelector('img'), cap = box.querySelector('p');
  box.querySelector('button').addEventListener('click', function () { box.close(); });
  box.addEventListener('click', function (e) { if (e.target === box) box.close(); });
  document.querySelectorAll('.photo:not(.cad) img').forEach(function (img) {
    img.classList.add('zoomable');
    img.addEventListener('click', function () {
      big.src = img.currentSrc || img.src; big.alt = img.alt;
      var fc = img.closest('figure').querySelector('figcaption');
      cap.textContent = fc ? fc.textContent : '';
      box.showModal();
    });
  });
})();

// Console button. On the hosted site it just follows the link to console.html.
// In the single-file version the console's HTML is bundled in, and it's written into a
// new same-origin window (not a blob: URL) so the browser still allows Web Bluetooth there.
(function () {
  var src = document.getElementById('console-src');
  if (!src) return;
  function consoleHTML() {
    var bin = atob(src.textContent.trim()), bytes = new Uint8Array(bin.length);
    for (var i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
    return new TextDecoder().decode(bytes);
  }
  window.openConsole = function () {
    var w = window.open('', '_blank');
    if (!w) return false;
    w.document.open();
    w.document.write(consoleHTML().replace('<head>', '<head><meta name="from-portfolio" content="1">'));
    w.document.close();
    return true;
  };
  document.querySelectorAll('[data-console]').forEach(function (a) {
    a.addEventListener('click', function (e) {
      if (window.openConsole()) e.preventDefault();
    });
  });
})();
