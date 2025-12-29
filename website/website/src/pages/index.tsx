import type {ReactNode} from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import { motion } from 'framer-motion';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();

  const containerVariants = {
    hidden: { opacity: 0 },
    visible: {
      opacity: 1,
      transition: {
        staggerChildren: 0.2
      }
    }
  };

  const itemVariants = {
    hidden: { y: 20, opacity: 0 },
    visible: {
      y: 0,
      opacity: 1,
      transition: {
        duration: 0.5,
        ease: [0.25, 0.46, 0.45, 0.94]
      }
    }
  };

  return (
    <motion.header
      className="hero hero--primary bg-[#0B0E14] text-[#E6E8EB]"
      initial={{ opacity: 0 }}
      animate={{ opacity: 1 }}
      transition={{ duration: 0.8 }}
    >
      {/* Animated grid background */}
      <div className="absolute inset-0 overflow-hidden">
        <div className="absolute inset-0 bg-[radial-gradient(#1F2937_1px,transparent_1px)] [background-size:16px_16px] opacity-30"></div>
        <div className="absolute inset-0 bg-[radial-gradient(#00E5FF_1px,transparent_1px)] [background-size:64px_64px] opacity-10"></div>
      </div>

      <div className="container padding-horiz--md relative z-10">
        <div className="row">
          <div className="col col--6 padding-vert--lg">
            <motion.div
              variants={containerVariants}
              initial="hidden"
              animate="visible"
            >
              <motion.div variants={itemVariants}>
                <Heading as="h1" className="hero__title text-4xl md:text-5xl font-bold text-[#00E5FF]">
                  {siteConfig.title}
                </Heading>
              </motion.div>
              <motion.div variants={itemVariants}>
                <p className="hero__subtitle text-xl md:text-2xl mt-4 text-[#9AA4B2]">
                  {siteConfig.tagline}
                </p>
              </motion.div>
              <motion.div variants={itemVariants} className="margin-top--lg">
                <div className="flex flex-col sm:flex-row gap-4">
                  <Link
                    className="button button--secondary button--lg bg-[#00E5FF] text-[#0B0E14] hover:bg-[#1AE8FF] transition-colors"
                    to="/docs/chapters/chapter1/section1-digital-ai-to-physical-ai">
                    Start Reading
                  </Link>
                  <Link
                    className="button button--outline button--lg text-[#00E5FF] border-[#00E5FF] hover:bg-[#00E5FF] hover:text-[#0B0E14] transition-colors"
                    to="/docs/intro">
                    Explore Features
                  </Link>
                </div>
              </motion.div>
            </motion.div>
          </div>
          <div className="col col--6 padding-vert--lg flex items-center justify-center">
            <motion.div
              className="relative"
              initial={{ scale: 0.8, rotateY: -15 }}
              animate={{ scale: 1, rotateY: 0 }}
              transition={{ duration: 0.8, ease: "easeOut" }}
            >
              {/* Futuristic humanoid robot illustration */}
              <div className="relative w-64 h-64 mx-auto">
                {/* Robot body with gradient */}
                <motion.div
                  className="absolute w-24 h-32 bg-gradient-to-b from-[#121826] to-[#0B0E14] rounded-lg mx-auto left-1/2 transform -translate-x-1/2 border border-[#1F2937]"
                  animate={{
                    y: [0, -5, 0],
                  }}
                  transition={{
                    duration: 3,
                    repeat: Infinity,
                    repeatType: "reverse",
                    ease: "easeInOut"
                  }}
                />

                {/* Robot head */}
                <motion.div
                  className="absolute w-16 h-16 bg-gradient-to-b from-[#121826] to-[#0B0E14] rounded-full mx-auto left-1/2 transform -translate-x-1/2 -top-4 border border-[#1F2937]"
                  animate={{
                    y: [0, -3, 0],
                  }}
                  transition={{
                    duration: 2.5,
                    repeat: Infinity,
                    repeatType: "reverse",
                    ease: "easeInOut",
                    delay: 0.2
                  }}
                >
                  {/* Eyes with accent color */}
                  <div className="absolute top-4 left-3 w-2 h-2 bg-[#00E5FF] rounded-full"></div>
                  <div className="absolute top-4 right-3 w-2 h-2 bg-[#00E5FF] rounded-full"></div>
                  {/* LED indicator */}
                  <motion.div
                    className="absolute top-6 left-1/2 transform -translate-x-1/2 w-1 h-1 bg-[#7C4DFF] rounded-full"
                    animate={{ scale: [1, 1.5, 1] }}
                    transition={{
                      duration: 1.5,
                      repeat: Infinity,
                      repeatType: "reverse",
                      ease: "easeInOut"
                    }}
                  />
                </motion.div>

                {/* Robot arms */}
                <motion.div
                  className="absolute w-6 h-20 bg-gradient-to-b from-[#121826] to-[#0B0E14] rounded-lg -left-6 top-8 origin-top border border-[#1F2937]"
                  animate={{ rotate: [-5, 5, -5] }}
                  transition={{
                    duration: 4,
                    repeat: Infinity,
                    repeatType: "reverse",
                    ease: "easeInOut"
                  }}
                />
                <motion.div
                  className="absolute w-6 h-20 bg-gradient-to-b from-[#121826] to-[#0B0E14] rounded-lg -right-6 top-8 origin-top border border-[#1F2937]"
                  animate={{ rotate: [5, -5, 5] }}
                  transition={{
                    duration: 4,
                    repeat: Infinity,
                    repeatType: "reverse",
                    ease: "easeInOut",
                    delay: 0.3
                  }}
                />

                {/* Robot legs */}
                <div className="absolute w-6 h-16 bg-gradient-to-b from-[#121826] to-[#0B0E14] rounded-lg -left-2 bottom-0 border border-[#1F2937]"></div>
                <div className="absolute w-6 h-16 bg-gradient-to-b from-[#121826] to-[#0B0E14] rounded-lg left-10 bottom-0 border border-[#1F2937]"></div>

                {/* Floating neural network nodes */}
                {[...Array(8)].map((_, i) => (
                  <motion.div
                    key={i}
                    className="absolute w-2 h-2 bg-[#00E5FF] rounded-full opacity-70"
                    style={{
                      left: `${20 + (i * 10)}%`,
                      top: `${30 + (i * 5)}%`,
                    }}
                    animate={{
                      y: [0, -20, 0],
                      x: [0, Math.sin(i) * 10, 0],
                      scale: [1, 1.2, 1],
                      opacity: [0.7, 1, 0.7]
                    }}
                    transition={{
                      duration: 3 + i,
                      repeat: Infinity,
                      repeatType: "reverse",
                      ease: "easeInOut",
                      delay: i * 0.2,
                    }}
                  />
                ))}
              </div>

              {/* Floating tech elements */}
              <motion.div
                className="absolute -top-6 -right-6 text-4xl text-[#7C4DFF]"
                animate={{
                  rotate: [0, 10, 0],
                  scale: [1, 1.1, 1]
                }}
                transition={{
                  duration: 4,
                  repeat: Infinity,
                  repeatType: "reverse",
                  ease: "easeInOut"
                }}
              >
                ⚙️
              </motion.div>
              <motion.div
                className="absolute -bottom-6 -left-6 text-4xl text-[#00E5FF]"
                animate={{
                  rotate: [0, -10, 0],
                  scale: [1, 1.1, 1]
                }}
                transition={{
                  duration: 3.5,
                  repeat: Infinity,
                  repeatType: "reverse",
                  ease: "easeInOut",
                  delay: 0.5
                }}
              >
                🧠
              </motion.div>
            </motion.div>
          </div>
        </div>
      </div>
    </motion.header>
  );
}

function HomepageFeatures() {
  const {siteConfig} = useDocusaurusContext();

  const features = [
    {
      title: 'Physical AI Foundation',
      description: 'Understand how AI moves from digital to physical realms, bridging the gap between algorithms and real-world interaction.',
      icon: '🤖'
    },
    {
      title: 'Humanoid Design',
      description: 'Explore the engineering challenges and solutions behind creating humanoid robots that can navigate and interact with human environments.',
      icon: '🦾'
    },
    {
      title: 'Embodied Intelligence',
      description: 'Learn how robots develop intelligence through physical interaction with the world, similar to how humans learn through experience.',
      icon: '🧠'
    }
  ];

  const containerVariants = {
    hidden: { opacity: 0 },
    visible: {
      opacity: 1,
      transition: {
        staggerChildren: 0.2
      }
    }
  };

  const itemVariants = {
    hidden: { y: 20, opacity: 0 },
    visible: {
      y: 0,
      opacity: 1,
      transition: {
        duration: 0.5,
        ease: "easeOut"
      }
    }
  };

  return (
    <section className="margin-vert--lg bg-[#0B0E14]">
      <div className="container">
        <motion.div
          className="text-center margin-bottom--lg"
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.6 }}
        >
          <Heading as="h2" className="text-3xl font-bold text-[#00E5FF]">
            What You'll Learn
          </Heading>
          <p className="text-lg text-[#9AA4B2] mt-2">
            Comprehensive coverage of the latest in Physical AI and Humanoid Robotics
          </p>
        </motion.div>

        <motion.div
          variants={containerVariants}
          initial="hidden"
          whileInView="visible"
          viewport={{ once: true, margin: "-100px" }}
          className="row"
        >
          {features.map((feature, index) => (
            <motion.div
              key={index}
              variants={itemVariants}
              className="col col--4 margin-bottom--lg"
            >
              <div className="text--center padding-horiz--md">
                <motion.div
                  className="text-4xl margin-bottom--sm text-[#00E5FF]"
                  whileHover={{ scale: 1.2, rotate: 5 }}
                  transition={{ type: "spring", stiffness: 300 }}
                >
                  {feature.icon}
                </motion.div>
                <Heading as="h3" className="text-xl font-semibold text-[#E6E8EB]">
                  {feature.title}
                </Heading>
                <p className="text-[#9AA4B2]">
                  {feature.description}
                </p>
              </div>
            </motion.div>
          ))}
        </motion.div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="An interactive textbook on Physical AI & Humanoid Robotics">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
